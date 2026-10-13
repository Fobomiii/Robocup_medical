#!/bin/bash
set -e

# 启动 Livox Mid360 ROS2 驱动
# 话题: /livox/lidar (PointCloud2, 10Hz)
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
MID360_STARTUP_TIMEOUT_S=${MEDICAL_MID360_STARTUP_TIMEOUT_S:-30}

case "$MID360_STARTUP_TIMEOUT_S" in
    ''|*[!0-9]*)
        echo "MEDICAL_MID360_STARTUP_TIMEOUT_S must be a non-negative integer, got: $MID360_STARTUP_TIMEOUT_S" >&2
        exit 2
        ;;
esac

ros2 launch livox_ros_driver2 my_mid360_launch.py &
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
        echo "Mid360 launch did not stop after SIGINT; sending SIGTERM" >&2
        kill -TERM "$launch_pid" 2>/dev/null || true
        deadline=$((SECONDS + 2))
        while kill -0 "$launch_pid" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do
            sleep 0.2
        done
    fi
    if kill -0 "$launch_pid" 2>/dev/null; then
        echo "Mid360 launch did not stop after SIGTERM; sending SIGKILL" >&2
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

deadline=$((SECONDS + MID360_STARTUP_TIMEOUT_S))
lidar_ready=false
while kill -0 "$launch_pid" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do
    if timeout 3s ros2 topic echo --once /livox/lidar --field header \
        >/dev/null 2>&1; then
        lidar_ready=true
        break
    fi
    sleep 1
done

if [ "$lidar_ready" != true ]; then
    echo "Mid360 launched but /livox/lidar produced no sample; restarting driver" >&2
    stop_launch
    trap - INT TERM EXIT
    exit 69
fi

echo "Mid360 point cloud startup verified"

set +e
wait "$launch_pid"
launch_status=$?
set -e
trap - INT TERM EXIT
exit "$launch_status"
