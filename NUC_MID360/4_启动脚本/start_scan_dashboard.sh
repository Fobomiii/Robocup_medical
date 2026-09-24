#!/bin/bash
set -e

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
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1

mkdir -p "$HOME/.cache"
exec 9>"$HOME/.cache/medical-scan-dashboard.lock"
flock -n 9 || exit 0

exec ros2 run obstacle_detector scan_dashboard
