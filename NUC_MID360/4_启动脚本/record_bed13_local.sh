#!/usr/bin/env bash
set -Eeo pipefail

pause_and_exit() {
    local status="$1"
    shift
    printf '%s\n' "$*"
    read -r -p "Press Enter to close this window..." || true
    exit "$status"
}

set +u
if [ -f "$HOME/.config/medical-navigation.env" ]; then
    set -a
    source "$HOME/.config/medical-navigation.env"
    set +a
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

if [ ! -r /opt/ros/humble/setup.bash ]; then
    pause_and_exit 1 "ROS 2 Humble setup was not found."
fi
source /opt/ros/humble/setup.bash

if [ ! -r "$HOME/livox_ws/install/setup.bash" ]; then
    pause_and_exit 1 "NUC workspace setup was not found: $HOME/livox_ws/install/setup.bash"
fi
source "$HOME/livox_ws/install/setup.bash"
set -u

lock_dir="${XDG_RUNTIME_DIR:-/tmp}"
exec 9>"$lock_dir/medical-bed13-rosbag.lock"
if ! flock -n 9; then
    pause_and_exit 1 "Another Bed13 rosbag recorder is already running."
fi

stamp="$(date +%Y%m%d_%H%M%S)"
output="$HOME/bed13_$stamp"
topics=(
    /evaluation
    /cmd_vel_nav
    /cmd_vel
    /cmd_vel_home_limited
    /cmd_vel_collision_safe
    /cmd_vel_safe
    /medical_nav/omni_escape_cmd_vel
    /medical_nav/omni_escape_status
    /medical_nav/braking_shadow_cmd_vel
    /medical_nav/braking_shadow_status
    /medical_nav/task_state
    /medical_nav/navigator_status
    /medical_nav/bed_approach_status
    /medical_nav/home_approach_status
    /medical_nav/field_boundary_status
    /medical_nav/cone_compensation_status
    /medical_nav/lidar_filter_status
    /medical_nav/lidar_safety_filter_status
    /medical_nav/robot_pose
    /medical_nav/bridge_cmd_debug
    /medical_nav/bridge_status
    /medical_nav/stm32_ready
    /medical_nav/mission_epoch
    /medical_nav/stp23l
    /medical_nav/stp23l/left
    /medical_nav/stp23l/front
    /medical_nav/stp23l/right
    /medical_nav/corner_speed_status
    /medical_nav/reference_speed_profile
    /medical_nav/reference_speed_status
    /medical_nav/execution_path
    /medical_nav/execution_path_status
    /medical_nav/wheel_odom
    /medical_nav/wheel_diagnostics
    /odom
    /plan
    /plan_smoothed
    /transformed_global_plan
    /trajectories
    /local_costmap/costmap_raw
    /local_costmap/costmap_updates
    /local_costmap/published_footprint
    /global_costmap/costmap_raw
    /global_costmap/costmap_updates
    /global_costmap/published_footprint
    /speed_limit
    /navigate_to_pose/_action/status
    /tf
    /tf_static
)

printf 'Recording on this NUC: %s\n' "$output"
printf '%s\n' "Press Ctrl+C once to stop and finalize the rosbag."
printf '\n'

set +e
ros2 bag record --storage sqlite3 -o "$output" "${topics[@]}"
status=$?
set -e

if [ -r "$output/metadata.yaml" ]; then
    printf '\nRosbag saved successfully: %s\n' "$output"
else
    printf '\nRosbag stopped without metadata: %s\n' "$output" >&2
    if [ "$status" -eq 0 ]; then
        status=1
    fi
fi
read -r -p "Press Enter to close this window..." || true
exit "$status"
