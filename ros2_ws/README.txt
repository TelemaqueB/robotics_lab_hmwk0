# Homework 2 - Robotics Lab 2025

## Installation

### Prerequisites
- Ubuntu 22.04
- ROS 2 Humble

### Clone and build
\`\`\`bash
cd ~/ros2_ws/src
git clone <your_repo_url>
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
\`\`\`

## Usage

### Part 1: Kinematic Control

#### 1a: Parameters and Launch
\`\`\`bash
ros2 launch ros2_kdl_package kdl_control.launch.py
\`\`\`

#### 1b: Null Space Controller
ros2 launch ros2_kdl_package kdl_control.launch.py ctrl:=velocity_ctrl_null

#### 1c: Action Server/Client
# Terminal 1
ros2 run ros2_kdl_package trajectory_action_server

# Terminal 2
ros2 run ros2_kdl_package trajectory_action_client

### Part 2: Vision-Based Control

#### 2a-2b: Vision Controller

# Terminal 1: Launch simulation
ros2 launch iiwa_description iiwa_aruco.launch.py

# Terminal 2: Launch vision controller
ros2 launch ros2_kdl_package kdl_control.launch.py ctrl:=vision_ctrl

# Terminal 3: View camera feed
ros2 run rqt_image_view rqt_image_view

#### 2c: Move Marker Service
ros2 service call /move_marker iiwa_description/srv/MoveMarker "{target_pose: {position: {x: 0.7, y: 0.3, z: 1.2}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"

## Author
- Télémaque BUZZI

---
