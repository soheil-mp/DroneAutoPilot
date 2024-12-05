#!/bin/bash

# Unset PYTHONPATH to ensure we're using system Python
unset PYTHONPATH

# Source ROS2
source /opt/ros/humble/setup.bash

# Set Python to system Python
export PYTHON_EXECUTABLE=/usr/bin/python3

# Clean build
rm -rf ~/ros2_ws/build/drone_nav_rl_interfaces
rm -rf ~/ros2_ws/install/drone_nav_rl_interfaces
rm -rf ~/ros2_ws/log/latest_build

# Build the package
cd ~/ros2_ws
colcon build --packages-select drone_nav_rl_interfaces 