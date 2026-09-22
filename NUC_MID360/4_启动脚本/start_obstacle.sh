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

SERIAL_PORT=${1:-/dev/medical_stm32}
DRY_RUN=${2:-${MEDICAL_NAV_DRY_RUN:-false}}
SCAN_CAMERA=${MEDICAL_SCAN_CAMERA:-/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0}
SCANNER_ENABLED=${MEDICAL_SCANNER_ENABLED:-true}

case "$DRY_RUN" in
    true|false) ;;
    *)
        echo "MEDICAL_NAV_DRY_RUN must be true or false, got: $DRY_RUN" >&2
        exit 2
        ;;
esac

case "$SCANNER_ENABLED" in
    true|false) ;;
    *)
        echo "MEDICAL_SCANNER_ENABLED must be true or false, got: $SCANNER_ENABLED" >&2
        exit 2
        ;;
esac

echo "Starting Nav2: serial=$SERIAL_PORT dry_run=$DRY_RUN scanner=$SCANNER_ENABLED camera=$SCAN_CAMERA"
exec ros2 launch obstacle_detector obstacle.launch.py \
    serial_port:="$SERIAL_PORT" dry_run:="$DRY_RUN" \
    scanner_enabled:="$SCANNER_ENABLED" scan_camera:="$SCAN_CAMERA"
