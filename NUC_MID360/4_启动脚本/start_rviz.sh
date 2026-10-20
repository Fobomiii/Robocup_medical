#!/bin/bash
set -e

RVIZ_CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/medical-rviz"
mkdir -p "$RVIZ_CACHE_DIR"
exec > >(tee -a "$RVIZ_CACHE_DIR/startup.log") 2>&1

unset PYTHONHOME QT_QPA_PLATFORM_PLUGIN_PATH QT_PLUGIN_PATH
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"

NAV_ENV="$HOME/.config/medical-navigation.env"
if [ -r "$NAV_ENV" ]; then
    set -a
    source "$NAV_ENV"
    set +a
fi

RVIZ_ENV="$HOME/.config/medical-rviz.env"
if [ -r "$RVIZ_ENV" ]; then
    set -a
    source "$RVIZ_ENV"
    set +a
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$HOME/livox_ws/install/livox_ros_driver2/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM:-1}
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_ENABLE_HIGHDPI_SCALING=0
export QT_SCALE_FACTOR=${MEDICAL_RVIZ_SCALE:-1}
export QT_FONT_DPI=96
unset QT_SCREEN_SCALE_FACTORS QT_DEVICE_PIXEL_RATIO

CONFIG="$HOME/medical_nav.rviz"
RUNTIME_CONFIG="$RVIZ_CACHE_DIR/medical_nav.screen.rviz"
if [ ! -r "$CONFIG" ] || [ ! -r "$HOME/prepare_rviz_config.py" ]; then
    printf '%s\n' 'RViz configuration or prepare_rviz_config.py is missing; install all RViz launcher files.' >&2
    exit 1
fi

/usr/bin/python3 "$HOME/prepare_rviz_config.py" "$CONFIG" "$RUNTIME_CONFIG"
exec rviz2 -d "$RUNTIME_CONFIG"
