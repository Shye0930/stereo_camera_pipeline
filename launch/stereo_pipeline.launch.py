from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix # Import get_package_prefix
import os

def generate_launch_description():
    # [INFO] Get the path to the config directory
    package_share_directory = get_package_share_directory('stereo_camera_pipeline')

    config_dir = os.path.join(package_share_directory, 'config')
    calibration_file_path = os.path.join(package_share_directory, 'config', 'stereo_calibration.yaml')    

    package_prefix = get_package_prefix('stereo_camera_pipeline')

    calibration_file_path =  '/home/shye/Desktop/projects/fyp/config/stereo/stereo_calibration.yaml'

    return LaunchDescription([
        Node(
            package='stereo_camera_pipeline',
            # Use the new console script name
            executable='stereo_processor_node',
            name='stereo_pipeline_node', # Give the combined node a suitable name
            output='screen',
            parameters=[
                {'left_camera_url': 'http://192.168.68.60'},
                {'right_camera_url': 'http://192.168.68.62'},
                {'calibration_file': calibration_file_path}
            ],
        ),
        # [INFO] Node for publishing raw stereo images from IP cameras
        # Node(
        #     package='stereo_camera_pipeline',
        #     executable='stereo_ip_publisher',
        #     name='stereo_ip_publisher_node',
        #     output='screen',
        #     parameters=[
        #         # Replace with your actual camera IP addresses
        #         {'left_camera_url': 'http://192.168.68.60'},
        #         {'right_camera_url': 'http://192.168.68.62'},
        #     ]
        # ),

        # Node for rectifying and publishing images
        # Node(
        #     package='stereo_camera_pipeline',
        #     executable='stereo_rectifier_node',
        #     name='stereo_rectifier_node',
        #     output='screen',
        #     parameters=[
        #         {'calibration_file': calibration_file_path}
        #     ],
        #     # Remap input topics to match the output of the stereo_ip_publisher
        #     remappings=[
        #         ('/stereo/left/image_raw', '/stereo/left/image_raw'),
        #         ('/stereo/right/image_raw', '/stereo/right/image_raw'),
        #     ]
        # )
    ])