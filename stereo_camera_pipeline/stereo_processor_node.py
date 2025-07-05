import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import threading
import queue
import requests
import time
import os
# get_package_share_directory is no longer directly used for default, but can stay if other parts of a larger package might use it
from ament_index_python.packages import get_package_share_directory 

# --- CUSTOM YAML CONSTRUCTOR (keep this section as it is) ---
def opencv_matrix_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    rows = mapping['rows']
    cols = mapping['cols']
    dt = mapping['dt']
    data = mapping.get('data', [])
    expected_size = rows * cols
    if len(data) != expected_size:
        raise ValueError(
            f"Invalid data size for opencv-matrix: expected {expected_size} elements, got {len(data)}. "
            f"Data: {data}, rows: {rows}, cols: {cols}"
        )
    if dt == 'd':
        dtype = np.float64
    elif dt == 'f':
        dtype = np.float32
    elif dt == 'i':
        dtype = np.int32
    else:
        raise ValueError(f"Unsupported OpenCV matrix data type: {dt}")
    return np.array(data, dtype=dtype).reshape(rows, cols)

yaml.Loader.add_constructor('tag:yaml.org,2002:opencv-matrix', opencv_matrix_constructor)
# --- END CUSTOM YAML CONSTRUCTOR ---

# CameraStreamThread class (remains unchanged)
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
        self.frame_lock = threading.Lock()    
        
        if self.url and self.url.startswith('http'):
            self.set_flash(10)
        else:
            self.node.get_logger().warn(f"Invalid or empty URL provided for {self.name} camera. Skipping flash control.")

    def set_flash(self, intensity):
        intensity = max(0, min(255, intensity))
        try:
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
        self.stream_url = self.url + ":81/stream"
        self.cap = cv2.VideoCapture(self.stream_url)

        if not self.cap.isOpened():
            self.node.get_logger().error(f"Failed to open {self.name} stream from {self.stream_url}.")
            self.running = False
            return
        
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
                self.cap.release()
                time.sleep(0.1)
                self.cap = cv2.VideoCapture(self.stream_url)
                if not self.cap.isOpened():
                    self.node.get_logger().error(f"Reconnection to {self.name} failed. Stopping thread.")
                    self.running = False
                    break
                else:
                    self.node.get_logger().info(f"Successfully reconnected to {self.name} camera.")
            
            time.sleep(0.001)

        self.cap.release()
        self.node.get_logger().info(f"{self.name} camera thread stopped.")

    def get_frame(self):
        with self.frame_lock:
            return self.ret, self.latest_frame.copy() if self.latest_frame is not None else None

    def stop(self):
        self.set_flash(0)
        self.running = False


class StereoProcessorNode(Node):
    def __init__(self):
        super().__init__('stereo_processor_node')

        # --- Parameters ---
        self.declare_parameter('left_camera_url', 'http://192.168.68.60')
        self.declare_parameter('right_camera_url', 'http://192.168.68.62')


        self.declare_parameter('calibration_file', '')
        user_file = self.get_parameter('calibration_file').get_parameter_value().string_value

        final_path = ''

        # Priority 1: User-provided
        if user_file:
            final_path = user_file
            self.get_logger().info(f"Using user-provided calibration file: {final_path}")

        # Priority 2: Package share directory
        else:
            try:
                share_dir = get_package_share_directory('stereo_camera_pipeline')
                package_file = os.path.join(share_dir, 'config', 'stereo_calibration.yaml')
                if os.path.exists(package_file):
                    final_path = package_file
                    self.get_logger().info(f"Using calibration file from package share directory: {final_path}")
                else:
                    self.get_logger().warn(f"Calibration file not found in package share directory at: {package_file}")
            except PackageNotFoundError:
                self.get_logger().warn("Package 'stereo_camera_pipeline' not found.")
            except Exception as e:
                self.get_logger().error(f"Unexpected error while locating calibration file: {e}")

        # Final update
        self.set_parameters([Parameter('calibration_file', Parameter.Type.STRING, final_path)])
        self.calibration_file = final_path
        
        # No default path is provided here; it MUST be passed as an argument.
        # self.declare_parameter('calibration_file', '/home/shye/Desktop/projects/fyp/config/stereo/stereo_calibration.yaml') 
        # self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value

        self.left_url = self.get_parameter('left_camera_url').get_parameter_value().string_value
        self.right_url = self.get_parameter('right_camera_url').get_parameter_value().string_value
       

        self.get_logger().info(f"Using Left Camera URL: {self.left_url}")
        self.get_logger().info(f"Using Right Camera URL: {self.right_url}")
        # Message to user that calibration file is expected as argument
        self.get_logger().info(f"Calibration file parameter received: '{self.calibration_file}'. This should be an absolute path.")

        # --- CV Bridge and Rectification Maps ---
        self.cv_bridge = CvBridge()
        self.M1l, self.M2l = None, None
        self.M1r, self.M2r = None, None

        self.raw_left_image_pub = self.create_publisher(Image, '/stereo/left/image_raw', 10)
        self.raw_right_image_pub = self.create_publisher(Image, '/stereo/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/stereo/right/camera_info', 10)
        
        self.pub_rectified_left = self.create_publisher(Image, '/stereo/left/rectified_images', 10)
        self.pub_rectified_right = self.create_publisher(Image, '/stereo/right/rectified_images', 10)

        # --- Camera Stream Threads ---
        self.left_cam_thread = CameraStreamThread(self.left_url, "left", self)
        self.right_cam_thread = CameraStreamThread(self.right_url, "right", self)

        self.left_cam_thread.start()
        self.right_cam_thread.start()

        # Call the function to load calibration and initialize maps
        self.load_and_init_rectification()

        # --- Main Timer for Processing ---
        self.timer = self.create_timer(0.01, self.process_and_publish_frames)

        self.get_logger().info("Stereo Processor Node started and ready to process images.")

    def load_and_init_rectification(self):
        abs_calibration_path = self.calibration_file

        # --- MODIFIED VALIDATION ---
        if not abs_calibration_path:
            self.get_logger().error("[FATAL] Calibration file path is empty. It must be provided via the 'calibration_file' parameter (e.g., --ros-args -p calibration_file:=/full/path/to/stereo_calibration.yaml).")
            rclpy.shutdown() # Shutdown node if critical dependency is missing
            return
        
        if not os.path.exists(abs_calibration_path):
            self.get_logger().error(f"[FATAL] Calibration file not found at: {abs_calibration_path}. Please provide a valid absolute path.")
            rclpy.shutdown() # Shutdown node if file doesn't exist
            return
        # --- END MODIFIED VALIDATION ---

        try:
            self.get_logger().info(f"[INFO] Loading calibration from: {abs_calibration_path}")
            
            with open(abs_calibration_path, 'r') as f:
                settings = yaml.load(f, Loader=yaml.Loader)
            
            K_l = settings['LEFT.K']
            D_l = settings['LEFT.D']
            R_l = settings['LEFT.R']
            P_l = settings['LEFT.P']
            
            K_r = settings['RIGHT.K']
            D_r = settings['RIGHT.D']
            R_r = settings['RIGHT.R']
            P_r = settings['RIGHT.P']

            cols_l = settings['LEFT.width']
            rows_l = settings['LEFT.height']
            cols_r = settings['RIGHT.width']
            rows_r = settings['RIGHT.height']

            P_l_3x3 = P_l[:3, :3]
            P_r_3x3 = P_r[:3, :3]
            
            self.M1l, self.M2l = cv2.initUndistortRectifyMap(
                K_l, D_l, R_l, P_l_3x3, (cols_l, rows_l), cv2.CV_32F)
            self.M1r, self.M2r = cv2.initUndistortRectifyMap(
                K_r, D_r, R_r, P_r_3x3, (cols_r, rows_r), cv2.CV_32F)

            self.get_logger().info("[INFO] Calibration parameters loaded and rectification maps initialized.")

        except yaml.YAMLError as e:
            self.get_logger().error(f"[FATAL] Error parsing calibration YAML file '{abs_calibration_path}': {e}")
            rclpy.shutdown()
            return
        except KeyError as e:
            self.get_logger().error(f"[FATAL] Missing key '{e}' in calibration file '{abs_calibration_path}'. "
                                     f"Please check if the YAML structure matches expected keys like 'LEFT.K', 'LEFT.width', etc.")
            rclpy.shutdown()
            return
        except Exception as e:
            self.get_logger().error(f"[FATAL] Unexpected error loading calibration or initializing maps from '{abs_calibration_path}': {e}")
            rclpy.shutdown()
            return

    def process_and_publish_frames(self):
        ret_l, frame_l = self.left_cam_thread.get_frame()
        ret_r, frame_r = self.right_cam_thread.get_frame()

        if ret_l and ret_r:
            now = self.get_clock().now().to_msg()

            # --- Publish Raw Images and Info ---
            msg_l_raw = self.cv_bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
            msg_r_raw = self.cv_bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')

            msg_l_raw.header.stamp = now
            msg_r_raw.header.stamp = now
            msg_l_raw.header.frame_id = 'left_camera'
            msg_r_raw.header.frame_id = 'right_camera'

            self.raw_left_image_pub.publish(msg_l_raw)
            self.raw_right_image_pub.publish(msg_r_raw)

            h, w = frame_l.shape[:2]
            
            cam_info_l = CameraInfo()
            cam_info_l.header.stamp = now
            cam_info_l.header.frame_id = 'left_camera'
            cam_info_l.width = w
            cam_info_l.height = h
            # NOTE: For full CameraInfo, you would populate K, D, R, P from the loaded calibration data here.
            # This current code only sets header, width, and height.
            self.left_info_pub.publish(cam_info_l)

            cam_info_r = CameraInfo()
            cam_info_r.header.stamp = now
            cam_info_r.header.frame_id = 'right_camera'
            cam_info_r.width = w
            cam_info_r.height = h
            self.right_info_pub.publish(cam_info_r)

            # --- Perform Rectification and Publish Rectified Images ---
            if self.M1l is not None and self.M1r is not None:
                try:
                    im_left_rectified = cv2.remap(frame_l, self.M1l, self.M2l, cv2.INTER_LINEAR)
                    im_right_rectified = cv2.remap(frame_r, self.M1r, self.M2r, cv2.INTER_LINEAR)

                    ros_img_left_rectified = self.cv_bridge.cv2_to_imgmsg(im_left_rectified, encoding='bgr8') 
                    ros_img_right_rectified = self.cv_bridge.cv2_to_imgmsg(im_right_rectified, encoding='bgr8')

                    ros_img_left_rectified.header = msg_l_raw.header
                    ros_img_right_rectified.header = msg_r_raw.header

                    self.pub_rectified_left.publish(ros_img_left_rectified)
                    self.pub_rectified_right.publish(ros_img_right_rectified)
                except Exception as e:
                    self.get_logger().error(f"[ERROR] Error during rectification or publishing rectified images: {e}")
            else:
                self.get_logger().warn("[WARN] Rectification maps not initialized (likely due to missing/invalid calibration file). Skipping rectification.")
        else:
            self.get_logger().warn("Failed to read one or both frames from camera threads. Waiting for data...")

    def destroy_node(self):
        self.get_logger().info("Stopping camera stream threads...")
        self.left_cam_thread.stop()
        self.right_cam_thread.stop()
        self.left_cam_thread.join()
        self.right_cam_thread.join()
        self.get_logger().info("Camera stream threads stopped.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()