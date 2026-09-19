#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/gp-pcie/livox_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/home/gp-pcie/livox_ws/install/livox_ros_driver2/lib:${LD_LIBRARY_PATH}
export DISPLAY=${DISPLAY:-:0}

CONFIG=/home/gp-pcie/medical_nav.rviz
sleep 2
exec rviz2 -d "$CONFIG"
