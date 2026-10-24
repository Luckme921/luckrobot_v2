#!/usr/bin/env bash
# Source in each navigation / diagnostics / RViz terminal. No process is started.
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/config/fastdds_udp.xml"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
