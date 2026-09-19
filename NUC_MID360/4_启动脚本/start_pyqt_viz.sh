#!/bin/bash
# 启动旧的 PyQt 场地监控窗口
source /opt/ros/humble/setup.bash
source /home/gp-pcie/livox_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/home/gp-pcie/livox_ws/install/livox_ros_driver2/lib:$LD_LIBRARY_PATH
export DISPLAY=${DISPLAY:-:0}
cd /home/gp-pcie
exec python3 /home/gp-pcie/navigation_viz.py
