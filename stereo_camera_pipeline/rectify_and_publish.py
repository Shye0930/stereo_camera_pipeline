import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os

# Import message_filters for image synchronization
import message_filters

# Add this import to resolve package share directory
from ament_index_python.packages import get_package_share_directory

# --- CUSTOM YAML CONSTRUCTOR (essential for loading OpenCV matrices from YAML) ---
def opencv_matrix_constructor(loader, node):
    """
    Custom YAML constructor for OpenCV matrix (tag:yaml.org,2002:opencv-matrix).
    This allows loading matrices like K, D, R, P from a YAML calibration file
    directly into NumPy arrays.
    """
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

# Register the custom constructor with the YAML loader
yaml.Loader.add_constructor('tag:yaml.org,2002:opencv-matrix', opencv_matrix_constructor)
# --- END CUSTOM YAML CONSTRUCTOR ---


class StereoRectifierNode(Node):
    """
    A ROS 2 node that subscribes to raw stereo images, rectifies them using
    pre-loaded calibration parameters, and publishes the rectified images.
    """
    def __init__(self):
        super().__init__('stereo_rectifier_node')


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


        self.get_logger().info(f"Using calibration file: {self.calibration_file}")

        # --- CV Bridge and Rectification Maps ---
        self.cv_bridge = CvBridge()
        self.M1l, self.M2l = None, None # Maps for left camera rectification
        self.M1r, self.M2r = None, None # Maps for right camera rectification

  
        self.pub_rectified_left = self.create_publisher(Image, '/stereo/left/rectified_images', 10)
        self.pub_rectified_right = self.create_publisher(Image, '/stereo/right/rectified_images', 10)
        self.get_logger().info("Publishing rectified images on '/stereo/left/rectified_images' and '/stereo/right/rectified_images'")

        # --- Subscribers for Raw Images with Synchronization ---
        # Create subscribers for raw left and right images
        self.sub_left_raw = message_filters.Subscriber(self, Image, '/stereo/left/image_raw')
        self.sub_right_raw = message_filters.Subscriber(self, Image, '/stereo/right/image_raw')
        self.get_logger().info("Subscribing to raw images on '/stereo/left/image_raw' and '/stereo/right/image_raw'")

        # Use ApproximateTimeSynchronizer to synchronize left and right image messages
        # This ensures that we process stereo pairs that were captured at roughly the same time.
        # The 'slop' parameter defines the maximum time difference between messages to be considered synchronized.
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left_raw, self.sub_right_raw],
            queue_size=10,
            slop=0.1 # 100ms tolerance for time synchronization
        )
        # Register the callback function to be called when a synchronized pair of messages arrives
        self.ts.registerCallback(self.stereo_image_callback)
        self.get_logger().info("Image synchronization (ApproximateTimeSynchronizer) set up.")

        # Load calibration and initialize rectification maps upon node startup
        self.load_and_init_rectification()

        self.get_logger().info("Stereo Rectifier Node started and ready to rectify images.")

    def load_and_init_rectification(self):
        """
        Loads stereo calibration parameters from the specified YAML file
        and initializes the rectification maps (M1l, M2l, M1r, M2r)
        required by cv2.remap.
        """
        abs_calibration_path = self.calibration_file

        if not abs_calibration_path:
            self.get_logger().error("[ERROR] Calibration file path is empty. Please provide it via 'calibration_file' parameter (full absolute path).")
            rclpy.shutdown() # Shutdown the ROS context if calibration file is not provided
            return

        try:
            self.get_logger().info(f"[INFO] Loading calibration from: {abs_calibration_path}")

            with open(abs_calibration_path, 'r') as f:
                settings = yaml.load(f, Loader=yaml.Loader)

            # Access calibration matrices and image dimensions from the loaded YAML data
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

            # Extract the 3x3 projection matrix from the 3x4 projection matrix (P)
            # This 3x3 matrix is used by initUndistortRectifyMap as the new camera matrix.
            P_l_3x3 = P_l[:3, :3]
            P_r_3x3 = P_r[:3, :3]

            # Initialize the rectification maps for both left and right cameras.
            # These maps are used by cv2.remap to undistort and rectify images.
            self.M1l, self.M2l = cv2.initUndistortRectifyMap(
                K_l, D_l, R_l, P_l_3x3, (cols_l, rows_l), cv2.CV_32F)
            self.M1r, self.M2r = cv2.initUndistortRectifyMap(
                K_r, D_r, R_r, P_r_3x3, (cols_r, rows_r), cv2.CV_32F)

            self.get_logger().info("[INFO] Calibration parameters loaded and rectification maps initialized successfully.")

        except FileNotFoundError:
            self.get_logger().error(f"[ERROR] Calibration file not found at: {abs_calibration_path}")
            self.get_logger().error("Please ensure the provided 'calibration_file' parameter points to a valid absolute path.")
            rclpy.shutdown()
            return
        except yaml.YAMLError as e:
            self.get_logger().error(f"[ERROR] Error parsing calibration YAML file '{abs_calibration_path}': {e}")
            rclpy.shutdown()
            return
        except KeyError as e:
            self.get_logger().error(f"[ERROR] Missing key '{e}' in calibration file '{abs_calibration_path}'. "
                                     f"Please check if the YAML structure matches expected keys like 'LEFT.K', 'LEFT.width', etc.")
            rclpy.shutdown()
            return
        except Exception as e:
            self.get_logger().error(f"[ERROR] Unexpected error loading calibration or initializing maps from '{abs_calibration_path}': {e}")
            rclpy.shutdown()
            return

    def stereo_image_callback(self, msg_left: Image, msg_right: Image):
        """
        Callback function for synchronized left and right raw image messages.
        It converts ROS Image messages to OpenCV format, rectifies them,
        and publishes the rectified images.
        """
        # Ensure rectification maps are initialized before proceeding
        if self.M1l is None or self.M1r is None:
            self.get_logger().warn("Rectification maps not initialized. Skipping image rectification.")
            return

        try:
            # Convert ROS Image messages to OpenCV images (NumPy arrays)
            frame_l = self.cv_bridge.imgmsg_to_cv2(msg_left, desired_encoding='bgr8')
            frame_r = self.cv_bridge.imgmsg_to_cv2(msg_right, desired_encoding='bgr8')

            # Perform rectification using the pre-computed maps
            im_left_rectified = cv2.remap(frame_l, self.M1l, self.M2l, cv2.INTER_LINEAR)
            im_right_rectified = cv2.remap(frame_r, self.M1r, self.M2r, cv2.INTER_LINEAR)

            # Convert rectified OpenCV images back to ROS Image messages
            ros_img_left_rectified = self.cv_bridge.cv2_to_imgmsg(im_left_rectified, encoding='bgr8')
            ros_img_right_rectified = self.cv_bridge.cv2_to_imgmsg(im_right_rectified, encoding='bgr8')

            # Copy header information (timestamp and frame_id) from the raw messages
            # This is crucial for downstream processing that relies on timing and coordinate frames.
            ros_img_left_rectified.header = msg_left.header
            ros_img_right_rectified.header = msg_right.header

            # Publish the rectified images
            self.pub_rectified_left.publish(ros_img_left_rectified)
            self.pub_rectified_right.publish(ros_img_right_rectified)

        except Exception as e:
            self.get_logger().error(f"[ERROR] Error during image conversion, rectification, or publishing: {e}")

def main(args=None):
    """
    Main function to initialize and run the StereoRectifierNode.
    """
    rclpy.init(args=args) # Initialize the ROS 2 client library
    node = StereoRectifierNode() # Create an instance of the node
    try:
        rclpy.spin(node) # Keep the node alive, processing callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Stereo Rectifier Node stopped by user (Ctrl+C).")
    finally:
        node.destroy_node() # Clean up the node resources
        rclpy.shutdown() # Shut down the ROS 2 client library

if __name__ == '__main__':
    main()
