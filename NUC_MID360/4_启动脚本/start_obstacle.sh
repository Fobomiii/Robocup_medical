#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/gp-pcie/livox_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

SERIAL_PORT=${1:-/dev/ttyUSB0}
DRY_RUN=${2:-${MEDICAL_NAV_DRY_RUN:-false}}

case "$DRY_RUN" in
    true|false) ;;
    *)
        echo "MEDICAL_NAV_DRY_RUN must be true or false, got: $DRY_RUN" >&2
        exit 2
        ;;
esac

echo "Starting Nav2: serial=$SERIAL_PORT dry_run=$DRY_RUN"
exec ros2 launch obstacle_detector obstacle.launch.py \
    serial_port:="$SERIAL_PORT" dry_run:="$DRY_RUN"
