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
# The competition display is managed in physical pixels. Desktop DPI probing
# can otherwise enlarge the frameless window and fixed fonts beyond the panel.
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_ENABLE_HIGHDPI_SCALING=0
export QT_SCALE_FACTOR=${MEDICAL_DASHBOARD_SCALE:-1}
unset QT_SCREEN_SCALE_FACTORS
unset QT_FONT_DPI
unset QT_DEVICE_PIXEL_RATIO

mkdir -p "$HOME/.cache"
exec ros2 run obstacle_detector scan_dashboard
