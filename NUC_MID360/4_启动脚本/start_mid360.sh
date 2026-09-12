#!/bin/bash
# 启动 Livox Mid360 ROS2 驱动
# 话题: /livox/lidar (PointCloud2, 10Hz)
source /opt/ros/humble/setup.bash
source /home/gp-pcie/livox_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/home/gp-pcie/livox_ws/install/livox_ros_driver2/lib:$LD_LIBRARY_PATH

LAUNCH=/home/gp-pcie/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/my_mid360_launch.py
ros2 launch "$LAUNCH"
