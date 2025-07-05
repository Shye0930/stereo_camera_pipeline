import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter # Import Parameter for parameter handling
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64 # Import Float64 for publishing the rate
from cv_bridge import CvBridge
import cv2
import threading
import queue
import requests
import time # Import time for sleep

class CameraStreamThread(threading.Thread):
    def __init__(self, url, name, node):
        super().__init__()
        self.url = url
        self.name = name
        self.node = node
        self.cap = None
        self.latest_frame = None
        self.ret = False
        self.running = True
        self.frame_lock = threading.Lock() # To protect access to latest_frame
        
        # Attempt to set flash off only if URL seems valid
        if self.url and self.url.startswith('http'):
            self.set_flash(0)
        else:
            self.node.get_logger().warn(f"Invalid or empty URL provided for {self.name} camera. Skipping flash control.")

    def set_flash(self, intensity):
        """HTTP control method"""
        intensity = max(0, min(255, intensity))
        try:
            # Assuming the control endpoint is on port 80 and path /control
            url = f"{self.url}:80/control?var=led_intensity&val={intensity}"
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                self.intensity = intensity
                self.node.get_logger().info(f"Set {self.name} LED intensity to {intensity}.")
                return True
            else:
                self.node.get_logger().warn(f"Failed to set {self.name} LED intensity. Status: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            self.node.get_logger().error(f"HTTP LED intensity control for {self.name} failed: {e}")
            return False
        except Exception as e:
            self.node.get_logger().error(f"Unexpected error in {self.name} LED intensity control: {e}")
            return False
        

    def run(self):
        self.node.get_logger().info(f"Connecting to {self.name} camera at {self.url}:81/stream...")
        # Assuming the stream endpoint is on port 81 and path /stream
        self.stream_url = self.url + ":81/stream"
        self.cap = cv2.VideoCapture(self.stream_url)

        if not self.cap.isOpened():
            self.node.get_logger().error(f"Failed to open {self.name} stream from {self.stream_url}.")
            self.running = False
            return

        # Optional: Try to reduce buffer size if supported by the camera/stream
        # Note: cv2.CAP_PROP_BUFFERSIZE is not widely supported across all backends
        # or IP camera streams. It's more effective for local webcams with certain drivers.
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        
        # Consider a small sleep to prevent busy-looping if frames are not available immediately
        # or to allow the camera stream to stabilize.
        time.sleep(0.5) 

        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.ret = True
            else:
                self.node.get_logger().warn(f"Failed to read frame from {self.name} camera. Attempting to re-open stream...")
                self.ret = False
                # Simple reconnection logic: release and re-open
                self.cap.release()
                time.sleep(0.1) # Short delay before retrying
                self.cap = cv2.VideoCapture(self.stream_url)
                if not self.cap.isOpened():
                    self.node.get_logger().error(f"Reconnection to {self.name} failed. Stopping thread.")
                    self.running = False # Stop trying if reconnection fails repeatedly
                    break
                else:
                    self.node.get_logger().info(f"Successfully reconnected to {self.name} camera.")
            
            # Add a small sleep to yield CPU and prevent high CPU usage,
            # especially if the stream is slow or if get_frame() is called less frequently.
            # This sleep helps balance between getting the latest frame and resource usage.
            time.sleep(0.001) 

        self.cap.release()
        self.node.get_logger().info(f"{self.name} camera thread stopped.")

    def get_frame(self):
        with self.frame_lock:
            # Return a copy to avoid external modification while this thread updates it
            # Ensure latest_frame is not None before copying
            return self.ret, self.latest_frame.copy() if self.latest_frame is not None else None

    def stop(self):
        self.set_flash(0) # Turn off flash when stopping
        self.running = False


class StereoCameraPublisher(Node):

    def __init__(self):
        super().__init__('stereo_camera_publisher')

        # Declare parameters for camera URLs with default values
        self.declare_parameter('left_camera_url', 'http://192.168.68.60')
        self.declare_parameter('right_camera_url', 'http://192.168.68.62')

        # Get parameter values
        self.left_url = self.get_parameter('left_camera_url').get_parameter_value().string_value
        self.right_url = self.get_parameter('right_camera_url').get_parameter_value().string_value

        self.get_logger().info(f"Using Left Camera URL: {self.left_url}")
        self.get_logger().info(f"Using Right Camera URL: {self.right_url}")

        self.left_image_pub = self.create_publisher(Image, '/stereo/left/image_raw', 10)
        self.right_image_pub = self.create_publisher(Image, '/stereo/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/stereo/right/camera_info', 10)
        

        self.bridge = CvBridge()

        # Create dedicated threads for each camera
        self.left_cam_thread = CameraStreamThread(self.left_url, "left", self)
        self.right_cam_thread = CameraStreamThread(self.right_url, "right", self)

        self.left_cam_thread.start()
        self.right_cam_thread.start()

        # Timer to periodically get frames from threads and publish
        # This timer will run very frequently (100 Hz), pushing frames as fast as they are available.
        self.timer = self.create_timer(0.01, self.publish_stereo_frames) 


    def publish_stereo_frames(self):
        ret_l, frame_l = self.left_cam_thread.get_frame()
        ret_r, frame_r = self.right_cam_thread.get_frame()

        if ret_l and ret_r:
            now = self.get_clock().now().to_msg()

            msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
            msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')

            msg_l.header.stamp = now
            msg_r.header.stamp = now
            msg_l.header.frame_id = 'left_camera'
            msg_r.header.frame_id = 'right_camera'


            self.left_image_pub.publish(msg_l)
            self.right_image_pub.publish(msg_r)

            # Publish dummy CameraInfo (for calibration to work)
            if frame_l is not None:
                h, w = frame_l.shape[:2]
                cam_info_l = CameraInfo()
                cam_info_l.header.stamp = now
                cam_info_l.header.frame_id = 'left_camera'
                cam_info_l.width = w
                cam_info_l.height = h
                self.left_info_pub.publish(cam_info_l)

            if frame_r is not None:
                h, w = frame_r.shape[:2]
                cam_info_r = CameraInfo()
                cam_info_r.header.stamp = now
                cam_info_r.header.frame_id = 'right_camera'
                cam_info_r.width = w
                cam_info_r.height = h
                self.right_info_pub.publish(cam_info_r)
                
        else:
            self.get_logger().warn("Failed to read one or both frames from camera threads. Waiting for data...")


    def destroy_node(self):
        self.get_logger().info("Stopping camera stream threads...")
        self.left_cam_thread.stop()
        self.right_cam_thread.stop()
        self.left_cam_thread.join() # Wait for the thread to finish
        self.right_cam_thread.join() # Wait for the thread to finish
        self.get_logger().info("Camera stream threads stopped.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()