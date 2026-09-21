#!/bin/bash
# 启动原始点云调参窗口
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"

NAV_ENV="$HOME/.config/medical-navigation.env"
if [ -r "$NAV_ENV" ]; then
    set -a
    source "$NAV_ENV"
    set +a
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$HOME/livox_ws/install/livox_ros_driver2/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY=${DISPLAY:-:0}
cd "$HOME"
exec python3 "$HOME/lidar_viz.py"
