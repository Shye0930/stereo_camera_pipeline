# ROS 2 Stereo Camera Processor

A ROS 2 node for streaming, rectifying, and publishing images from a stereo IP camera pair (e.g. ESP32-CAMs over HTTP MJPEG). Uses a calibration file for stereo rectification.

## Overview

- Streams from two IP cameras
- Loads stereo calibration from a YAML file
- Publishes raw and rectified images
- Works with ESP32-CAM LED flash control

## Features

- Multi-threaded video capture
- Stereo rectification using OpenCV
- Automatic calibration file resolution:
  1. User-provided `calibration_file` parameter
  2. Fallback to package share directory (`config/stereo_calibration.yaml`)

## Available Executables

This package provides the following launchable ROS 2 nodes:

- `stereo_ip_publisher` → `stereo_camera_pipeline.publish_stereo:main`
- `stereo_rectifier_node` → `stereo_camera_pipeline.rectify_and_publish:main`
- `stereo_processor_node` → `stereo_camera_pipeline.stereo_processor_node:main`

Use any of these with `ros2 run`:

```bash
ros2 run stereo_camera_pipeline stereo_processor_node
```

## Topics

- `/stereo/left/image_raw`
- `/stereo/right/image_raw`
- `/stereo/left/camera_info`
- `/stereo/right/camera_info`
- `/stereo/left/rectified_images`
- `/stereo/right/rectified_images`

## Parameters

- `left_camera_url` (default: `http://192.168.68.60`)
- `right_camera_url` (default: `http://192.168.68.62`)
- `calibration_file` (absolute path to YAML)

## Running the Node

```bash
ros2 run stereo_camera_pipeline stereo_processor_node --ros-args \
  -p left_camera_url:="http://192.168.1.100" \
  -p right_camera_url:="http://192.168.1.101" \
  -p calibration_file:="/path/to/stereo_calibration.yaml"
```

### Running the Node via launch file 
Edit the launch file to launch the necessary node
```bash
ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py
```

## Rebuilding the package
```sh
rm -rf build log install
colcon build --symlink-install --packages-select stereo_camera_pipeline
```

