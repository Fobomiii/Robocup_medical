#!/bin/bash
# 启动可视化调试工具（需要桌面环境）
source /opt/ros/humble/setup.bash
source /home/gp-pcie/livox_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/home/gp-pcie/livox_ws/install/livox_ros_driver2/lib:$LD_LIBRARY_PATH
cd /home/gp-pcie
python3 /home/gp-pcie/lidar_viz.py
