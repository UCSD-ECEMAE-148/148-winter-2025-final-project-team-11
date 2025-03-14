# Robocar Visual Pursuit Package

A ROS2 package for autonomous vehicle pursuit using computer vision and DepthAI. This package enables a robocar to detect and follow another vehicle using visual tracking.

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Nodes](#nodes)
  - [Car Detection Node](#car-detection-node)
  - [Lane Guidance Node](#lane-guidance-node)
  - [Parameter Tuner Node](#parameter-tuner-node)
- [Topics](#topics)
- [Launch Files](#launch-files)
- [Parameter Tuning](#parameter-tuning)
- [Troubleshooting](#troubleshooting)

## Features
- Real-time car detection using YOLO and DepthAI
- Adaptive PID-based steering control
- Dynamic throttle management
- Real-time parameter tuning interface
- Parameter persistence across launches
- Robust tracking with lost-frame handling

## Prerequisites
- ROS2
- DepthAI
- OpenCV
- Python 3.8+

## Installation
1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url> robocar_visual_pursuit_pkg
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select robocar_visual_pursuit_pkg
```

## Nodes

### Car Detection Node
The car detection node uses YOLO and DepthAI to detect vehicles in the camera feed and calculate their position relative to the robot.

**Parameters:**
- `confidence_threshold` (0.0-1.0): Confidence threshold for car detection
- `camera_centerline` (0.0-1.0): Normalized position of camera centerline
- `error_threshold` (0.0-1.0): Error threshold for steering control
- `iou_threshold` (0.0-1.0): IOU threshold for object detection
- `max_lost_frames` (0-30): Maximum number of frames to track lost object

**Topics:**
- Subscribes: `/camera/color/image_raw` (sensor_msgs/Image)
- Publishes: `/centroid` (std_msgs/Float32)

### Lane Guidance Node
The lane guidance node implements PID control for steering and adaptive throttle management based on tracking error.

**Parameters:**
- `Kp_steering` (0.0-5.0): Proportional gain for steering control
- `Ki_steering` (0.0-2.0): Integral gain for steering control
- `Kd_steering` (0.0-2.0): Derivative gain for steering control
- `max_throttle` (0.0-1.0): Maximum throttle value
- `min_throttle` (0.0-0.5): Minimum throttle value

**Topics:**
- Subscribes: `/centroid` (std_msgs/Float32)
- Publishes: `/cmd_vel` (geometry_msgs/Twist)

### Parameter Tuner Node
A dedicated node for real-time parameter tuning with persistent storage.

**Features:**
- Real-time parameter adjustment via rqt_reconfigure
- Parameter persistence across launches
- Automatic parameter forwarding to relevant nodes

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | Raw camera feed |
| `/centroid` | std_msgs/Float32 | Normalized target position (-1.0 to 1.0) |
| `/cmd_vel` | geometry_msgs/Twist | Vehicle control commands |

## Launch Files

### Car Pursuit Launch
Launch the complete visual pursuit system:
```bash
ros2 launch robocar_visual_pursuit_pkg car_pursuit_launch.py
```

This launches:
- Car detection node
- Lane guidance node
- Parameter tuner node
- VESC interface node

## Parameter Tuning

### Real-time Parameter Tuning
1. Launch the visual pursuit system:
```bash
ros2 launch robocar_visual_pursuit_pkg car_pursuit_launch.py
```

2. In a new terminal, launch the parameter tuning GUI:
```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

### Saving Parameters
Save current parameters to make them persistent:
```bash
ros2 service call /parameter_tuner_node/save_parameters rcl_interfaces/srv/SetParameters "{}"
```

### Configuration Files
The system uses two configuration files in the `config` directory:
- `default_params.yaml`: Default parameters that ship with the package
- `custom_params.yaml`: Your saved custom parameters

The system automatically loads `custom_params.yaml` if it exists, otherwise falls back to `default_params.yaml`.

To revert to default parameters, simply delete `custom_params.yaml` before launching the system.

## Troubleshooting

### Common Issues

#### No Car Detection
- Check if the DepthAI camera is properly connected
- Verify the model file exists in the correct location
- Adjust the `confidence_threshold` parameter
- Check camera exposure and lighting conditions

#### Unstable Following
- Tune PID parameters:
  1. Start with only P control (Ki=0, Kd=0)
  2. Increase Kp until oscillation occurs, then reduce by 50%
  3. Gradually increase Kd to reduce overshooting
  4. Add small Ki if steady-state error persists
- Adjust `error_threshold` and throttle parameters

#### Parameter Changes Not Persisting
- Ensure you've called the save_parameters service
- Check write permissions in the config directory
- Verify the custom_params.yaml file is being created
