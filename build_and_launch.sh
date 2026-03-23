#!/bin/bash

# ROS2 Build and Launch Script
# This script builds the embedded_systems_project package and launches mars_launch.py

cd ~/ros2_ws || { echo "Failed to navigate to ~/ros2_ws"; exit 1; }

echo "Building embedded_systems_project package..."
colcon build --packages-select embedded_systems_project

if [ $? -ne 0 ]; then
    echo "Build failed. Exiting."
    exit 1
fi

echo "Sourcing setup.bash..."
source ~/ros2_ws/install/setup.bash

echo "Launching mars_launch.py..."
ros2 launch embedded_systems_project mars_launch.py
