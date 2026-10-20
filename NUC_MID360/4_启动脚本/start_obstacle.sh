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

SERIAL_PORT=${1:-/dev/ttyUSB0}
DRY_RUN=${2:-${MEDICAL_NAV_DRY_RUN:-false}}
SCAN_CAMERA=${MEDICAL_SCAN_CAMERA:-/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0}
TELE_SCAN_CAMERA=${MEDICAL_TELE_SCAN_CAMERA:-/dev/v4l/by-id/usb-BLC-240823--A_SDYH-8P0P-video-index0}
SCANNER_ENABLED=${MEDICAL_SCANNER_ENABLED:-true}
HEALTH_BLE_ENABLED=${MEDICAL_HEALTH_BLE_ENABLED:-true}
HEALTH_BLE_DEVICE_NAME=${MEDICAL_HEALTH_BLE_DEVICE_NAME:-MedicalVitals-S3}
HEALTH_BLE_DEVICE_ADDRESS=${MEDICAL_HEALTH_BLE_DEVICE_ADDRESS:-}
STARTUP_DEVICE_WAIT_S=${MEDICAL_STARTUP_DEVICE_WAIT_S:-20}
LIDAR_STARTUP_WAIT_S=${MEDICAL_LIDAR_STARTUP_WAIT_S:-30}
NAV2_STARTUP_TIMEOUT_S=${MEDICAL_NAV2_STARTUP_TIMEOUT_S:-45}

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

case "$HEALTH_BLE_ENABLED" in
    true|false) ;;
    *)
        echo "MEDICAL_HEALTH_BLE_ENABLED must be true or false, got: $HEALTH_BLE_ENABLED" >&2
        exit 2
        ;;
esac

for value_name in STARTUP_DEVICE_WAIT_S LIDAR_STARTUP_WAIT_S NAV2_STARTUP_TIMEOUT_S; do
    value=${!value_name}
    case "$value" in
        ''|*[!0-9]*)
            echo "$value_name must be a non-negative integer, got: $value" >&2
            exit 2
            ;;
    esac
done

wait_for_startup_devices() {
    local deadline=$((SECONDS + STARTUP_DEVICE_WAIT_S))
    local missing=()
    while true; do
        missing=()
        local devices=("$SERIAL_PORT")
        if [ "$SCANNER_ENABLED" = true ]; then
            devices+=("$SCAN_CAMERA" "$TELE_SCAN_CAMERA")
        fi
        for device in "${devices[@]}"; do
            [ -e "$device" ] || missing+=("$device")
        done
        if [ ${#missing[@]} -eq 0 ]; then
            echo "Startup devices are present"
            return 0
        fi
        if [ "$SECONDS" -ge "$deadline" ]; then
            echo "Startup device wait expired; continuing with hot-plug retry: ${missing[*]}" >&2
            return 0
        fi
        sleep 1
    done
}

wait_for_lidar_sample() {
    local deadline=$((SECONDS + LIDAR_STARTUP_WAIT_S))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if timeout 3s ros2 topic echo --once /livox/lidar --field header \
            >/dev/null 2>&1; then
            echo "Mid360 point cloud is available"
            return 0
        fi
        sleep 1
    done
    echo "Mid360 readiness wait expired; restarting navigation startup" >&2
    return 1
}

wait_for_startup_devices
if ! wait_for_lidar_sample; then
    exit 69
fi

echo "Starting Nav2: serial=$SERIAL_PORT dry_run=$DRY_RUN scanner=$SCANNER_ENABLED camera=$SCAN_CAMERA tele_camera=$TELE_SCAN_CAMERA health_ble=$HEALTH_BLE_ENABLED"

HEALTH_BLE_ARGS=(
    health_ble_enabled:="$HEALTH_BLE_ENABLED"
    health_ble_device_name:="$HEALTH_BLE_DEVICE_NAME"
)
if [ -n "$HEALTH_BLE_DEVICE_ADDRESS" ]; then
    HEALTH_BLE_ARGS+=(
        health_ble_device_address:="$HEALTH_BLE_DEVICE_ADDRESS"
    )
fi

ros2 launch obstacle_detector obstacle.launch.py \
    serial_port:="$SERIAL_PORT" dry_run:="$DRY_RUN" \
    scanner_enabled:="$SCANNER_ENABLED" scan_camera:="$SCAN_CAMERA" \
    tele_scan_camera:="$TELE_SCAN_CAMERA" \
    "${HEALTH_BLE_ARGS[@]}" &
launch_pid=$!

stop_launch() {
    if ! kill -0 "$launch_pid" 2>/dev/null; then
        return
    fi

    kill -INT "$launch_pid" 2>/dev/null || true
    local deadline=$((SECONDS + 6))
    while kill -0 "$launch_pid" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do
        sleep 0.2
    done
    if kill -0 "$launch_pid" 2>/dev/null; then
        echo "ROS launch did not stop after SIGINT; sending SIGTERM" >&2
        kill -TERM "$launch_pid" 2>/dev/null || true
        deadline=$((SECONDS + 2))
        while kill -0 "$launch_pid" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do
            sleep 0.2
        done
    fi
    if kill -0 "$launch_pid" 2>/dev/null; then
        echo "ROS launch did not stop after SIGTERM; sending SIGKILL" >&2
        kill -KILL "$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" 2>/dev/null || true
}

handle_stop() {
    trap - INT TERM EXIT
    stop_launch
    exit 0
}

trap handle_stop INT TERM
trap stop_launch EXIT

if ! ros2 run obstacle_detector startup_probe \
    --timeout "$NAV2_STARTUP_TIMEOUT_S" --stable-samples 2; then
    echo "Authoritative navigation startup probe failed" >&2
    stop_launch
    trap - INT TERM EXIT
    exit 70
fi
if [ -n "${NOTIFY_SOCKET:-}" ]; then
    systemd-notify --ready --status="Navigation startup verified"
fi

set +e
wait "$launch_pid"
launch_status=$?
set -e
trap - INT TERM EXIT
exit "$launch_status"
