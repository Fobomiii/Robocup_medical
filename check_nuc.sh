#!/bin/bash
# Read-only Nav2 preflight for the NUC. This script never enables chassis motion.

ROS_SETUP=/opt/ros/humble/setup.bash
WS_SETUP="$HOME/livox_ws/install/setup.bash"

section() {
    printf '\n=== %s ===\n' "$1"
}

check_package() {
    if ros2 pkg prefix "$1" >/dev/null 2>&1; then
        printf '[OK]   %s\n' "$1"
    else
        printf '[MISS] %s\n' "$1"
    fi
}

if [ ! -f "$ROS_SETUP" ]; then
    echo "[FAIL] ROS 2 Humble setup not found: $ROS_SETUP"
    exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"
if [ -f "$WS_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$WS_SETUP"
else
    echo "[WARN] workspace setup not found: $WS_SETUP"
fi

NAV_ENV="$HOME/.config/medical-navigation.env"
if [ -r "$NAV_ENV" ]; then
    set -a
    source "$NAV_ENV"
    set +a
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ROS 2 environment hooks may probe optional unset variables while sourcing.
# Enable strict unset-variable checking only after both setup files have run.
set -u

section "ROS network isolation"
printf 'ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
printf 'ROS_LOCALHOST_ONLY=%s\n' "$ROS_LOCALHOST_ONLY"

section "Required ROS packages"
for package in \
    obstacle_detector \
    medical_clearance_planner \
    livox_ros_driver2 \
    nav2_bringup \
    nav2_mppi_controller \
    nav2_collision_monitor \
    nav2_navfn_planner \
    nav2_map_server \
    nav2_lifecycle_manager \
    nav2_rviz_plugins
do
    check_package "$package"
done

section "Clearance planner"
ros2 param get /planner_server GridBased.plugin 2>/dev/null || \
    echo "[WARN] planner plugin parameter unavailable"
for parameter in \
    blind_zone_enabled \
    blind_zone_angles_deg \
    blind_zone_half_width_deg \
    blind_zone_min_segment_length \
    blind_zone_min_lateral_offset \
    blind_zone_lateral_offset
do
    printf '%-40s ' "GridBased.$parameter"
    ros2 param get /planner_server "GridBased.$parameter" 2>/dev/null || echo "unavailable"
done
for parameter in \
    max_speed \
    crawl_speed \
    soft_decel \
    body_clearance \
    costmap_weight
do
    printf '%-28s ' "GridBased.$parameter"
    ros2 param get /planner_server "GridBased.$parameter" 2>/dev/null || echo "unavailable"
done

section "Normal acceleration and braking (ROS X forward, Y left)"
for parameter in feedback max_accel max_decel max_velocity; do
    printf '%-28s ' "velocity_smoother.$parameter"
    ros2 param get /velocity_smoother "$parameter" 2>/dev/null || echo "unavailable"
done

section "Motion lock"
if [ -f "$HOME/.config/medical-navigation.env" ]; then
    cat "$HOME/.config/medical-navigation.env"
else
    echo "[WARN] mode file missing; service fallback is dry-run=true"
fi
ros2 param get /stm32_bridge dry_run 2>/dev/null || \
    echo "[WARN] /stm32_bridge is not available"
ros2 param get /stm32_bridge enforce_task_gate 2>/dev/null || \
    echo "[WARN] bridge task gate is not available"

section "System services"
for service in mid360.service obstacle-detector.service; do
    state=$(systemctl is-active "$service" 2>/dev/null || true)
    printf '%-30s %s\n' "$service" "$state"
done

section "Serial device"
if compgen -G '/dev/ttyUSB*' >/dev/null; then
    ls -l /dev/ttyUSB*
else
    echo "[MISS] no /dev/ttyUSB* device"
fi

section "Scanner cameras"
primary_camera=${MEDICAL_SCAN_CAMERA:-/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0}
tele_camera=${MEDICAL_TELE_SCAN_CAMERA:-/dev/v4l/by-id/usb-BLC-240823--A_SDYH-8P0P-video-index0}
for camera in "$primary_camera" "$tele_camera"; do
    if [ -e "$camera" ]; then
        printf '[OK]   %s -> %s\n' "$camera" "$(readlink -f "$camera")"
    else
        printf '[MISS] %s\n' "$camera"
    fi
done

section "Required nodes"
nodes=$(ros2 node list 2>/dev/null || true)
for node in \
    /code_scanner \
    /stm32_bridge \
    /lidar_self_filter \
    /blind_zone_visualizer \
    /medical_navigator \
    /controller_server \
    /smoother_server \
    /planner_server \
    /behavior_server \
    /bt_navigator \
    /waypoint_follower \
    /velocity_smoother \
    /collision_monitor \
    /map_server \
    /robot_state_publisher
do
    if printf '%s\n' "$nodes" | grep -Fxq "$node"; then
        printf '[OK]   %s\n' "$node"
    else
        printf '[MISS] %s\n' "$node"
    fi
done

section "Required topics"
topics=$(ros2 topic list 2>/dev/null || true)
for topic in \
    /livox/lidar \
    /livox/lidar_filtered \
    /livox/lidar_nav \
    /livox/lidar_safety \
    /odom \
    /cmd_vel \
    /cmd_vel_safe \
    /medical_nav/robot_pose \
    /medical_nav/bridge_status \
    /medical_nav/stm32_ready \
    /medical_nav/scanner_status \
    /medical_nav/scanner_preview/compressed \
    /medical_nav/tele_scanner_preview/compressed \
    /medical_nav/lidar_filter_status \
    /medical_nav/lidar_safety_filter_status \
    /medical_nav/blind_zones \
    /medical_nav/blind_zone_status \
    /map \
    /global_costmap/costmap \
    /local_costmap/costmap \
    /plan \
    /medical_nav/plan \
    /tf \
    /tf_static
do
    if printf '%s\n' "$topics" | grep -Fxq "$topic"; then
        printf '[OK]   %s\n' "$topic"
    else
        printf '[MISS] %s\n' "$topic"
    fi
done

section "Lifecycle states"
for node in \
    /map_server \
    /controller_server \
    /smoother_server \
    /planner_server \
    /behavior_server \
    /bt_navigator \
    /waypoint_follower \
    /velocity_smoother \
    /collision_monitor
do
    printf '%-24s ' "$node"
    timeout 4 ros2 lifecycle get "$node" 2>/dev/null || echo "unavailable"
done

section "Bridge telemetry (one sample, max 4 s)"
timeout 4 ros2 topic echo --once /medical_nav/bridge_status 2>/dev/null || \
    echo "[WARN] no bridge status sample"

section "Scanner telemetry (one sample, max 4 s)"
timeout 4 ros2 topic echo --once /medical_nav/scanner_status 2>/dev/null || \
    echo "[WARN] no scanner status sample"

section "Point cloud header (one sample, max 4 s)"
timeout 4 ros2 topic echo --once /livox/lidar --field header 2>/dev/null || \
    echo "[WARN] no Mid360 point cloud sample"

section "Filtered point cloud (one sample, max 4 s)"
timeout 4 ros2 topic echo --once /livox/lidar_filtered --field header 2>/dev/null || \
    echo "[WARN] no filtered point cloud sample"
timeout 4 ros2 topic echo --once /medical_nav/lidar_filter_status 2>/dev/null || \
    echo "[WARN] no lidar self-filter status"
timeout 4 ros2 topic echo --once /livox/lidar_safety --field header 2>/dev/null || \
    echo "[WARN] no independent collision-safety point cloud"
timeout 4 ros2 topic echo --once /medical_nav/lidar_safety_filter_status 2>/dev/null || \
    echo "[WARN] no lidar safety-filter status"

section "X-drive lidar blind zones (one sample, max 4 s)"
timeout 4 ros2 topic echo --once /medical_nav/blind_zone_status 2>/dev/null || \
    echo "[WARN] no blind-zone status"

section "Pose and lidar rates (about 6 s each)"
echo "Expected after flashing the current STM32 firmware: /odom about 50 Hz"
timeout 6 ros2 topic hz /odom --window 100 2>/dev/null || true
echo "Expected from the Mid360 driver: /livox/lidar about 10 Hz"
timeout 6 ros2 topic hz /livox/lidar --window 50 2>/dev/null || true
echo "Expected after self filtering: /livox/lidar_filtered about 10 Hz"
timeout 6 ros2 topic hz /livox/lidar_filtered --window 50 2>/dev/null || true
echo "Expected on the lightweight safety path: /livox/lidar_safety about 10 Hz"
timeout 6 ros2 topic hz /livox/lidar_safety --window 50 2>/dev/null || true

section "Point cloud delay (about 6 s)"
echo "Delay should be small and stable; a large or negative value means clock mismatch."
timeout 6 ros2 topic delay /livox/lidar --window 50 2>/dev/null || true
echo "Filtered cloud keeps the raw Mid360 timestamp and should have the same delay."
timeout 6 ros2 topic delay /livox/lidar_filtered --window 50 2>/dev/null || true

section "Static map metadata (one sample, max 4 s)"
timeout 4 ros2 topic echo --once /map --field info 2>/dev/null || \
    echo "[WARN] no static map sample"

section "TF map -> base_link (max 4 s)"
tf_sample=$(timeout 4 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null || true)
if [ -n "$tf_sample" ]; then
    printf '%s\n' "$tf_sample"
else
    echo "[WARN] TF unavailable"
fi

section "Recent navigation log"
journalctl -u obstacle-detector.service --no-pager -n 20 2>/dev/null || true

cat <<'EOF'

Preflight is read-only. Keep MEDICAL_NAV_DRY_RUN=true until TF, point cloud,
costmaps and wheel-off-ground direction checks have all passed.
EOF
