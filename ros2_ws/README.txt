Télémaque BUZZI 

Homework 1 

You need 

# go in the workspace
cd ~/Robotics_Lab/homework_1/ros2_ws

# Source the file
source /opt/ros/humble/setup.bash
source install/setup.bash

# verify that the files compile
colcon build
source install/setup.bash

This will activate ros and check you can compile everything

To launch gazebo 

cd ~/Robotics_Lab/homework_1/ros2_ws
source install/setup.bash
ros2 launch armando_gazebo armando_world.launch.py

TO check the controlers:

cd ~/Robotics_Lab/homework_1/ros2_ws
source install/setup.bash
ros2 control list_controllers

And finally to launch the node

cd ~/Robotics_Lab/homework_1/ros2_ws
source install/setup.bash
ros2 run armando_control joint_position_publisher
