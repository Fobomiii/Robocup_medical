#!/bin/bash
# 启动障碍物检测节点（串口发送坐标）
# 用法: bash start_obstacle.sh [串口设备，默认 /dev/ttyUSB0]
source /opt/ros/humble/setup.bash
source /home/gp-pcie/livox_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

SERIAL_PORT=${1:-/dev/ttyUSB0}
ros2 launch obstacle_detector obstacle.launch.py serial_port:="$SERIAL_PORT"
