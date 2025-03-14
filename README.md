# <div align="center">Autonomous Police Car</div>
![Picture](markdown/image12.png){ width="800" height="600" style="display: block; margin: 0 auto" }
### <div align="center"> ECE 148 Final Project </div>
#### <div align="center"> Team 11 Winter 2025 </div>
<div align="center">
    
</div>

## Table of Contents
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#abstract">Abstract</a></li>
    <li><a href="#what-we-promised">What We Promised</a></li>
    <li><a href="#accomplishments">Accomplishments</a></li>
    <li><a href="#challenges">Challenges</a></li>
    <li><a href="#final-project-videos">Final Project Videos</a></li>
    <li><a href="#hardware">Hardware</a></li>
    <li><a href="#software">Software</a></li>
        <ul>
            <li><a href="#slam-simultaneous-localization-and-mapping">SLAM (Simultaneous Localization and Mapping)</a></li>
            <li><a href="#obstacle-avoidance">Obstacle Avoidance</a></li>
        </ul>
    <li><a href="#gantt-chart">Gantt Chart</a></li>
    <li><a href="#course-deliverables">Course Deliverables</a></li>
    <li><a href="#project-reproduction">Project Reproduction</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>

<hr>

## Team Members
MingWei Yeoh - ECE - 2025
Trevor 
Minh Quach
Jose 

<hr>

## Abstract
An realistic looking autonomous car that can activate lights when it detects a speeding car and autonomously pull it over or initiate a chase.

Past MAE projects haven’t tried to use car following for a real task, nor make their autonomous car LOOK like an actual car. 
Mechanically it will be a challenge to mount all the electronics inside of a small car body.
<hr>

## What We Promised

### Must Have
LEDs
Communication between LED controller and Jetson
Body shell Mounting
Chase the car
Yolo running on OAK-D Camera

### Nice to Have
Cool PCB
Clean Electronics mounting
A Siren
<hr>

## Accomplishments
- Follows car 
- LEDs work great 
- 150A Soft start power switch works great
<hr>

## Challenges
- Jetson not connecting to Wifi 
- ROS2 not configured correctly to run our nodes
- I2C reaching bugged state
<hr>

## Final Project Videos
@ MINH!!!!!!
<hr>

## Hardware 

Link to the OnShape [CAD](https://cad.onshape.com/documents/6c423dcbeb2588a9d7785133/w/5859bc8aee528f453679c661/e/83214a9dc768b66fa2069d19?renderMode=0&uiState=67d3caa0a720212af3f91fbd)

![alt text](markdown/image-2.png)

__Green__ - Body shell mounting method. It uses 8x3mm magnets. 
__Purple__ - Reference geometry from the car
__Black__ - Structural 3D Printed pieces
__Orange__ - 3D Printed Spacers

The design is super clean and allows the body shell to easily cover all the electronics. 

![alt text](markdown/image-3.png)

__Additional Hardware Necessary__

* Custom LED driver PCB
* Custom LED module
* 8x3mm Magnets
* 3mm bolts 
* Body shell

## PCB
### High Level Overview
* Embedded EPS32S3
* USB C Port
* Anti spark power switch capable of 150 A
* Buzzer (for siren)
* 2x High Powered led driver 
* 3x WS2815 Led Controllers
* Communication with Jetson via I2C
* 3 Spare GPIOs (if needed)
* Onboard VBAT -> 3V3 Buck regulator
![alt text](markdown/image-8.png)

__Power__
Snippet from the schematic. 3 Low RDson PMOS' act like the switch. When the on button is pressed, the 4s Lipo gets fed into the onboard buck regulator to power the microcontroller.
![alt text](markdown/image-9.png)


### Assembly
Hand Assembled. Theres a lot of parts
![alt text](markdown/image-4.png)  ![alt text](markdown/image-7.png)

### Bringup 
__LEDS__
![alt text](markdown/rgbbringup.gif)
![alt text](markdown/carlights.gif)
__I2C__
ESP32 acts as a peripheral device with address 0x55. It takes in a 2 byte command. 0x00 and 0x11. 
0x00 Turns on LEDs and 0x11 turns off LEDs.
![alt text](markdown/image11.png)

## Software - Robocar Visual Pursuit Package
A ROS2 package for autonomous vehicle pursuit using computer vision and DepthAI. This package enables a robocar to detect and follow another vehicle using visual tracking.

### Table of Contents
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

### Features
- Real-time car detection using YOLO and DepthAI
- Adaptive PID-based steering control
- Dynamic throttle management
- Real-time parameter tuning interface
- Parameter persistence across launches
- Robust tracking with lost-frame handling

### Prerequisites
- ROS2
- DepthAI
- OpenCV
- Python 3.8+

### Installation
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

### Nodes

#### Car Detection Node
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

#### Lane Guidance Node
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

#### Parameter Tuner Node
A dedicated node for real-time parameter tuning with persistent storage.

**Features:**
- Real-time parameter adjustment via rqt_reconfigure
- Parameter persistence across launches
- Automatic parameter forwarding to relevant nodes

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | Raw camera feed |
| `/centroid` | std_msgs/Float32 | Normalized target position (-1.0 to 1.0) |
| `/cmd_vel` | geometry_msgs/Twist | Vehicle control commands |

### Launch Files

#### Car Pursuit Launch
Launch the complete visual pursuit system:
```bash
ros2 launch robocar_visual_pursuit_pkg car_pursuit_launch.py
```

This launches:
- Car detection node
- Lane guidance node
- Parameter tuner node
- VESC interface node

### Parameter Tuning

#### Real-time Parameter Tuning
1. Launch the visual pursuit system:
```bash
ros2 launch robocar_visual_pursuit_pkg car_pursuit_launch.py
```

2. In a new terminal, launch the parameter tuning GUI:
```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

#### Saving Parameters
Save current parameters to make them persistent:
```bash
ros2 service call /parameter_tuner_node/save_parameters rcl_interfaces/srv/SetParameters "{}"
```

#### Configuration Files
The system uses two configuration files in the `config` directory:
- `default_params.yaml`: Default parameters that ship with the package
- `custom_params.yaml`: Your saved custom parameters

The system automatically loads `custom_params.yaml` if it exists, otherwise falls back to `default_params.yaml`.

To revert to default parameters, simply delete `custom_params.yaml` before launching the system.

### Troubleshooting

#### Common Issues

##### No Car Detection
- Check if the DepthAI camera is properly connected
- Verify the model file exists in the correct location
- Adjust the `confidence_threshold` parameter
- Check camera exposure and lighting conditions

##### Unstable Following
- Tune PID parameters:
  1. Start with only P control (Ki=0, Kd=0)
  2. Increase Kp until oscillation occurs, then reduce by 50%
  3. Gradually increase Kd to reduce overshooting
  4. Add small Ki if steady-state error persists
- Adjust `error_threshold` and throttle parameters

##### Parameter Changes Not Persisting
- Ensure you've called the save_parameters service
- Check write permissions in the config directory
- Verify the custom_params.yaml file is being created

## Gantt Chart
![alt text](markdown/image13.png)
## Course Deliverables
Here are our autonomous laps as part of our class deliverables:

* DonkeyCar Reinforcement Laps: https://youtube.com/shorts/6msSXXdx4cQ?feature=share
* GPS Laps: https://youtu.be/pAPebhBlGDo
* Lane Following: https://youtu.be/QzcPgDxxi5c

* All Slides: https://drive.google.com/drive/folders/1qeRSDj4K1PtiiYxJ_8OacECK8fxUN1bn?usp=sharing

## Acknowledgements
Special thanks to Alex and Winston!

<hr>