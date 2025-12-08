#!/bin/bash

# ===============================
#   Run TurtleBot3 in Gazebo
#   Empty world + model = burger
# ===============================

# Load ROS2 Humble environment
source /opt/ros/humble/setup.bash

# Load workspace if it exists
if [ -f "$HOME/wmr_smc_project/ros2_ws/install/setup.bash" ]; then
    source $HOME/wmr_smc_project/ros2_ws/install/setup.bash
fi

# TurtleBot3 model selection
export TURTLEBOT3_MODEL=burger

echo "====================================="
echo "Launching TurtleBot3 in empty Gazebo"
echo "Model: $TURTLEBOT3_MODEL"
echo "====================================="

# Launch Gazebo + TB3
ros2 launch turtlebot3_gazebo empty_world.launch.py