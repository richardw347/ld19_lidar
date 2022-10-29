#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" 
source "/opt/ros2_ws/install/local_setup.bash"

ros2 launch ld19_lidar lidar.launch.py
