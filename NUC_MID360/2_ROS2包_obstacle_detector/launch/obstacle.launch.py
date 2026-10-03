import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("obstacle_detector")
    nav2_share = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(package_share, "config", "nav2_params.yaml")
    default_map = os.path.join(package_share, "config", "static_map.yaml")
    field_map = os.path.join(package_share, "config", "field_map.yaml")
    scanner_params = os.path.join(package_share, "config", "scanner.yaml")
    default_bt_xml = os.path.join(
        package_share, "config", "navigate_to_pose_1hz.xml"
    )
    urdf_path = os.path.join(package_share, "config", "robot.urdf")
    with open(urdf_path, "r", encoding="utf-8") as stream:
        robot_description = stream.read()

    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
    bt_xml_file = LaunchConfiguration("bt_xml_file")
    dry_run = LaunchConfiguration("dry_run")
    enforce_task_gate = LaunchConfiguration("enforce_task_gate")
    scanner_enabled = LaunchConfiguration("scanner_enabled")
    scan_camera = LaunchConfiguration("scan_camera")
    tele_scan_camera = LaunchConfiguration("tele_scan_camera")
    health_ble_enabled = LaunchConfiguration("health_ble_enabled")
    health_ble_device_name = LaunchConfiguration("health_ble_device_name")
    health_ble_device_address = LaunchConfiguration("health_ble_device_address")

    # YAML cannot expand an ament package path itself. Rewrite only this leaf
    # parameter so bt_navigator receives the installed XML's absolute path.
    nav2_params_file = RewrittenYaml(
        source_file=params_file,
        param_rewrites={"default_nav_to_pose_bt_xml": bt_xml_file},
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("bt_xml_file", default_value=default_bt_xml),
            # Competition mode: allow real chassis output by default.
            DeclareLaunchArgument("dry_run", default_value="false"),
            # Reject Nav2 velocity outside the four delivery navigation states.
            DeclareLaunchArgument("enforce_task_gate", default_value="true"),
            DeclareLaunchArgument("scanner_enabled", default_value="true"),
            DeclareLaunchArgument("health_ble_enabled", default_value="true"),
            DeclareLaunchArgument(
                "health_ble_device_name", default_value="MedicalVitals-S3"
            ),
            DeclareLaunchArgument("health_ble_device_address", default_value=""),
            DeclareLaunchArgument(
                "scan_camera",
                default_value=(
                    "/dev/v4l/by-id/"
                    "usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0"
                ),
            ),
            DeclareLaunchArgument(
                "tele_scan_camera",
                default_value=(
                    "/dev/v4l/by-id/"
                    "usb-BLC-240823--A_SDYH-8P0P-video-index0"
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": False, "robot_description": robot_description}],
            ),
            Node(
                package="obstacle_detector",
                executable="lidar_self_filter",
                name="lidar_safety_filter",
                output="screen",
                parameters=[
                    {
                        # Collision Monitor gets its own short processing path:
                        # transform plus robot/noise rejection only. It must
                        # never wait for ground RANSAC or cone clustering.
                        "input_topic": "/livox/lidar",
                        "output_topic": "/livox/lidar_safety",
                        "status_topic": "/medical_nav/lidar_safety_filter_status",
                        "target_frame": "base_link",
                        # Static height rejection replaces expensive ground
                        # fitting on the emergency-stop path. Competition
                        # cones, people and boxes retain abundant returns.
                        "output_min_z": 0.12,
                        "output_max_z": 0.60,
                        "self_radius": 0.26,
                        "self_min_z": -0.05,
                        "self_max_z": 0.65,
                        "top_plate_filter_enabled": True,
                        "top_plate_center_x": 0.0,
                        "top_plate_center_y": 0.0,
                        "top_plate_side_length": 0.18,
                        "top_plate_margin": 0.03,
                        "top_plate_min_z": 0.52,
                        "top_plate_max_z": 0.65,
                        "arm_filter_enabled": True,
                        "arm_x_min": 0.10,
                        "arm_x_max": 0.40,
                        "arm_half_width": 0.14,
                        "arm_min_z": 0.45,
                        "arm_max_z": 0.82,
                        "ground_filter_enabled": False,
                        "reject_livox_noise": True,
                        "livox_noise_mask": 15,
                        "gate_on_stm32_ready": True,
                    }
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="obstacle_detector",
                executable="lidar_self_filter",
                name="lidar_self_filter",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/livox/lidar",
                        "output_topic": "/livox/lidar_filtered",
                        "target_frame": "base_link",
                        # Leave 50 mm below the 0.60 m acrylic top plate so
                        # grazing returns and chassis pitch cannot leak into
                        # cone recognition or either costmap.
                        "output_max_z": 0.55,
                        # Chassis radius 0.23 m plus 30 mm for frame/wheel returns.
                        "self_radius": 0.26,
                        "self_min_z": -0.05,
                        # The upright lidar mount/aluminium returns reach about
                        # z=0.54 m and are part of the robot, not obstacles.
                        "self_max_z": 0.65,
                        # Horizontal regular octagon centred on base_link. The
                        # physical side is 0.18 m with a flat side toward +X;
                        # offset every edge by 30 mm for mounting tolerance and
                        # unstable returns from the acrylic underside.
                        "top_plate_filter_enabled": True,
                        "top_plate_center_x": 0.0,
                        "top_plate_center_y": 0.0,
                        "top_plate_side_length": 0.18,
                        "top_plate_margin": 0.03,
                        "top_plate_min_z": 0.52,
                        "top_plate_max_z": 0.65,
                        # The mounted arm overhangs the 0.26 m self cylinder,
                        # so its returns used to be marked as an obstacle in
                        # front of the robot and could stall the chassis.
                        #
                        # Include the arm's measured volume plus its motion
                        # sweep and lidar jitter. Low returns below 0.45 m are
                        # deliberately preserved, so nearby cones, cabinets
                        # and people remain visible to the safety pipeline.
                        "arm_filter_enabled": True,
                        "arm_x_min": 0.10,
                        "arm_x_max": 0.40,
                        "arm_half_width": 0.14,
                        "arm_min_z": 0.45,
                        "arm_max_z": 0.82,
                        # Reject the slightly tilted floor plane before Nav2
                        # inflates individual floor returns into false walls.
                        "ground_filter_enabled": True,
                        "ground_distance_threshold": 0.04,
                        # Keep a 40 mm gap above Nav2's 80 mm marking floor so
                        # floating-point and pitch jitter cannot leak the floor
                        # directly into the obstacle layer.
                        "ground_removal_below": 0.06,
                        "ground_removal_above": 0.12,
                        "ground_max_tilt_deg": 12.0,
                        "ground_max_origin_height": 0.15,
                        "ground_min_inliers": 300,
                        "ground_candidate_min_z": -0.25,
                        "ground_candidate_max_z": 0.30,
                        # Fit outside 0.30 m to avoid chassis points, then apply
                        # the accepted plane from the 0.26 m self-filter edge.
                        "ground_removal_min_radius": 0.26,
                        "ground_min_radius": 0.30,
                        # Cover the global layer's complete 6.0 m marking range
                        # and its 0.5 m clearing-only band.
                        "ground_max_radius": 6.5,
                        "ground_ransac_iterations": 40,
                        # A single sparse Mid360 frame must not release the
                        # whole floor; briefly reuse the last accepted plane.
                        "ground_plane_hold_s": 1.0,
                        "reject_livox_noise": True,
                        "livox_noise_mask": 15,
                        # A deliberate STM32 RESET removes odom TF for about
                        # three seconds. Keep Livox alive, but do not forward
                        # clouds into Nav2 until bridge telemetry is stable.
                        "gate_on_stm32_ready": True,
                    }
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="obstacle_detector",
                executable="cone_footprint_compensator",
                name="cone_footprint_compensator",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/livox/lidar_filtered",
                        "output_topic": "/livox/lidar_nav",
                        # Estimate the axis from the lidar-facing cone surface,
                        # then keep 25 mm beyond the measured 0.155 m base to
                        # absorb sparse-scan axis error near a wheel.
                        "cone_height": 0.65,
                        "physical_base_radius": 0.155,
                        "base_radius": 0.18,
                        # Densify only the synthetic disk so both costmaps
                        # reliably retain the cone footprint at the rim.
                        "disk_spacing": 0.04,
                        "min_z": 0.08,
                        # Reject sparse/flat fragments, then merge adjacent
                        # returns before creating one disk per physical cone.
                        "min_points": 5,
                        "min_vertical_span": 0.12,
                        "center_merge_distance": 0.28,
                        # Reject compact clusters that never reach the ground.
                        # The mounted arm sits at 0.64-0.70 m and fits the
                        # cone cluster test, so without this it gets a false
                        # base disk at z=0.12 m in front of the robot.
                        "min_base_z": 0.35,
                        # Detect and expand competition cones up to two seconds
                        # ahead at the widened 3.0 m/s translation limit.
                        "max_range": 6.0,
                        # Hold only synthetic cone-base disks briefly. The
                        # source cloud remains frame-by-frame so people do
                        # not leave persistence trails.
                        "persistence_s": 0.60,
                        "persistence_frame": "odom",
                        "persistence_match_distance": 0.40,
                        # Livox timestamps can lead odom TF slightly. Accept
                        # latest TF only for a bounded future gap; a frozen or
                        # genuinely stale TF is still rejected.
                        "tf_future_fallback_enabled": True,
                        "tf_future_fallback_max_gap_s": 0.20,
                    }
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="obstacle_detector",
                executable="blind_zone_visualizer",
                name="blind_zone_visualizer",
                output="screen",
                parameters=[
                    {
                        # The four aluminium profiles follow the X-drive wheel
                        # axes and cast four body-fixed Mid360 shadow strips.
                        "frame_id": "base_link",
                        "marker_topic": "/medical_nav/blind_zones",
                        "status_topic": "/medical_nav/blind_zone_status",
                        "cmd_vel_topic": "/cmd_vel",
                        "angles_deg": [45.0, 135.0, -135.0, -45.0],
                        "origin_x_m": 0.0,
                        "origin_y_m": 0.0,
                        "start_distance_m": 0.23,
                        "width_m": 0.18,
                        # Length follows the 1.3 m/s^2 braking envelope; at
                        # 2 m/s it is about 2.29 m including delay and margin.
                        "min_length_m": 0.80,
                        "max_length_m": 2.50,
                        "braking_decel_m_s2": 1.30,
                        "reaction_time_s": 0.25,
                        "safety_margin_m": 0.25,
                        "half_width_deg": 7.0,
                        "cmd_timeout_s": 0.50,
                        "publish_rate_hz": 10.0,
                    }
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="obstacle_detector",
                executable="stm32_bridge",
                name="stm32_bridge",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "baud_rate": baud_rate,
                        "yaw_sign": -1.0,
                        "cmd_vel_topic": "/cmd_vel_safe",
                        "goal_handoff_hold_s": 0.5,
                        "navigator_following_hold_s": 0.3,
                        "navigator_status_timeout_s": 1.2,
                        # The STM32 is intentionally hard-reset between runs
                        # after the unpowered chassis has been pushed home.
                        # Require fresh stationary telemetry before reopening
                        # motion and the lidar navigation pipeline.
                        "stm32_telemetry_timeout_s": 0.25,
                        "stm32_recovery_hold_s": 0.75,
                        "stm32_recovery_max_linear_m_s": 0.03,
                        "stm32_recovery_max_angular_rad_s": 0.05,
                        # Continuously limit post-safety wheel acceleration.
                        # Stops and reductions still pass immediately.
                        "gate_release_wheel_accel_m_s2": 2.8,
                        "gate_release_yaw_radius_m": 0.25,
                        # Retained for deployed-parameter compatibility; the
                        # continuous limiter no longer needs re-arming.
                        "gate_release_rearm_drop_m_s": 0.25,
                        # Standard X-drive: body translation is capped on a
                        # 3.0 m/s circle and projected onto the 45-degree
                        # wheel axes with the physical 1/sqrt(2) factor.
                        "max_planar_speed_m_s": 3.0,
                        "max_wheel_speed_m_s": 3.0,
                        # MPPI supplies vx/vy. Bed 1, Bed 3 and Home replace
                        # MPPI wz with an independent HWT101CT yaw PID; nurse
                        # navigation keeps MPPI wz for QR scan viewpoints.
                        "heading_correction_max_wz_rad_s": 0.30,
                        "active_rotation_max_wz_rad_s": 0.80,
                        "active_rotation_linear_threshold_m_s": 0.05,
                        "heading_hold_enabled": True,
                        "heading_hold_task_states": [3, 6, 9],
                        "heading_hold_target_deg": 0.0,
                        "heading_hold_kp": 1.5,
                        "heading_hold_ki": 0.0,
                        "heading_hold_kd": 0.0,
                        "heading_hold_deadband_deg": 1.0,
                        "heading_hold_max_wz_rad_s": 0.30,
                        "heading_hold_integral_limit_rad_s": 0.05,
                        # Above 1.0 m/s in reverse, blend toward a stronger
                        # HWT-yaw PD profile. At 2.5 m/s it can arrest the
                        # measured negative yaw rate before error reaches the
                        # previous 7-12 degree range. Forward and low-speed
                        # behavior retain the validated base P controller.
                        "heading_hold_reverse_min_speed_m_s": 1.0,
                        "heading_hold_reverse_full_speed_m_s": 2.5,
                        "heading_hold_reverse_kp": 2.2,
                        # The 20 kg chassis needed about 0.36 s to reverse its
                        # measured yaw rate after the wheels had responded.
                        # Increase rate damping so correction starts before a
                        # large heading error develops; keep Kp and the yaw
                        # ceiling unchanged to avoid introducing oscillation.
                        "heading_hold_reverse_kd": 0.45,
                        "heading_hold_reverse_max_wz_rad_s": 0.40,
                        # Once full-speed reverse is reached, retain the
                        # reverse PD through braking until yaw error and yaw
                        # rate have both remained settled for 200 ms.
                        "heading_hold_reverse_hold_settle_s": 0.20,
                        "heading_hold_yaw_rate_deadband_rad_s": 0.02,
                        "nurse_scan_stop_linear_m_s": 0.03,
                        "nurse_scan_stop_angular_rad_s": 0.05,
                        "nurse_scan_stop_settle_s": 0.15,
                        "wheel_odom_timeout_s": 0.15,
                        "ekf_ops_position_std_m": 0.02,
                        "ekf_wheel_forward_std_m_s": 0.08,
                        "ekf_wheel_lateral_std_m_s": 0.16,
                        "ekf_wheel_yaw_std_rad_s": 0.12,
                        "ekf_linear_accel_std_m_s2": 1.5,
                        # Pre-generated bedside announcements at maximum
                        # ALSA mixer volume.
                        "tts_bed1_audio_file": "/home/fzurobot/Downloads/1_.mp3",
                        "tts_bed3_audio_file": "/home/fzurobot/Downloads/3_.mp3",
                        "tts_volume_percent": 100,
                        "tts_audio_device": "pulse",
                        # Keep a silent Pulse stream open while this service is
                        # running. Pulse still mixes calls and other desktop
                        # audio, while the amplifier no longer clips speech.
                        "tts_keepalive_enabled": True,
                        "tts_lead_silence_s": 0.0,
                        "tts_tail_silence_s": 0.0,
                        "tts_timeout_s": 8.0,
                        # Per-axis protocol guard after circular body-speed and
                        # physical X-drive wheel projection. STM32 repeats the
                        # same hard safety caps.
                        "max_speed_mm_s": 3000.0,
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "enforce_task_gate": ParameterValue(
                            enforce_task_gate, value_type=bool
                        ),
                        "field_config": field_map,
                        "bed_scan_activation_distance_m": 1.5,
                    }
                ],
            ),
            Node(
                package="obstacle_detector",
                executable="code_scanner",
                name="code_scanner",
                output="screen",
                condition=IfCondition(scanner_enabled),
                parameters=[
                    scanner_params,
                    {
                        "camera_device": scan_camera,
                        "tele_camera_device": tele_scan_camera,
                        "field_config": field_map,
                    },
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="obstacle_detector",
                executable="health_ble_bridge",
                name="health_ble_bridge",
                output="screen",
                condition=IfCondition(health_ble_enabled),
                parameters=[
                    {
                        "device_name": health_ble_device_name,
                        "device_address": health_ble_device_address,
                        "scan_timeout_s": 5.0,
                        "reconnect_delay_s": 1.0,
                        "connect_timeout_s": 10.0,
                        "forget_after_failures": 3,
                        "clear_stale_device": True,
                        "stale_timeout_s": 2.0,
                        "status_rate_hz": 5.0,
                    }
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
            # First-stage OPS slip experiment. Robust temporal scan matching
            # is independent of planning-only static-map annotations. It is
            # monitor-only and never publishes TF or changes chassis commands.
            Node(
                package="obstacle_detector",
                executable="lidar_odometry_guard",
                name="lidar_odometry_guard",
                output="screen",
                parameters=[params_file],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params_file, {"yaml_filename": map_file}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": autostart,
                        "node_names": ["map_server"],
                    }
                ],
            ),
            # Bring up the serial bridge, TF, map and lidar pipeline first.
            # Starting all Nav2 lifecycle nodes in the same CPU-heavy burst
            # can make Humble time out on smoother_server/change_state and
            # leave NavigateToPose permanently unavailable.
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(nav2_share, "launch", "navigation_launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": "false",
                            "autostart": autostart,
                            "params_file": nav2_params_file,
                            # Humble's navigation_launch.py embeds this value in a
                            # PythonExpression ("not <value>"), so it must use the
                            # Python boolean spelling rather than YAML's lowercase.
                            "use_composition": "False",
                            "use_respawn": "true",
                        }.items(),
                    )
                ],
            ),
            Node(
                package="obstacle_detector",
                executable="corner_speed_limiter",
                name="corner_speed_limiter",
                output="screen",
                parameters=[
                    {
                        # Prefer the smoothed, pruned path that MPPI actually
                        # follows. Keep the planner's raw path as a fallback
                        # during controller startup, recovery or goal handoff.
                        "raw_path_topic": "/plan",
                        "smoothed_path_topic": "/transformed_global_plan",
                        "pose_topic": "/medical_nav/robot_pose",
                        "output_topic": "/speed_limit",
                        "status_topic": "/medical_nav/corner_speed_status",
                        "controller_selector_topic": "/controller_selector",
                        "heading_hold_controller_id": "FollowPathHeadingHold",
                        "max_speed_m_s": 3.0,
                        # Bed 1/3 <-> Home uses FollowPathHeadingHold and may
                        # carry more speed through genuine path bends. Nurse
                        # and Bed 1 <-> Bed 3 retain the conservative profile.
                        "min_corner_speed_m_s": 1.0,
                        "lateral_accel_m_s2": 1.40,
                        "heading_hold_min_corner_speed_m_s": 1.10,
                        "heading_hold_lateral_accel_m_s2": 1.60,
                        # Match the stronger normal braking envelope so the
                        # limiter can retain cruise speed closer to a turn.
                        "braking_decel_m_s2": 1.80,
                        "lookahead_distance_m": 3.50,
                        # Average over 0.75 m on each side so short replanning
                        # kinks do not appear as persistent route corners.
                        "tangent_span_m": 0.75,
                        "sample_step_m": 0.10,
                        # Ignore the recurring 30-34 degree local-plan bends;
                        # genuine avoidance and Bed 1-3 turns remain limited.
                        "min_turn_angle_deg": 35.0,
                        "braking_margin_m": 0.05,
                        # The same X-shaped aluminium shadows used by the
                        # global planner. If the transformed MPPI path cuts a
                        # dogleg and again overlaps a blind strip for 0.6 m,
                        # retain a low-speed visibility fallback.
                        "blind_zone_enabled": True,
                        "blind_zone_angles_deg": [45.0, 135.0, -135.0, -45.0],
                        "blind_zone_half_width_deg": 7.0,
                        "blind_zone_min_overlap_m": 0.60,
                        "blind_zone_lookahead_m": 2.0,
                        "blind_zone_speed_m_s": 0.70,
                        "path_timeout_s": 1.50,
                        "pose_timeout_s": 0.50,
                        "smoothing_wait_s": 0.25,
                        "update_rate_hz": 10.0,
                    }
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="obstacle_detector",
                executable="home_approach_limiter",
                name="home_approach_limiter",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/cmd_vel",
                        "output_topic": "/cmd_vel_home_limited",
                        "pose_topic": "/medical_nav/robot_pose",
                        "task_state_topic": "/medical_nav/task_state",
                        "wheel_odom_topic": "/medical_nav/wheel_odom",
                        "status_topic": "/medical_nav/bed_approach_status",
                        "field_config": field_map,
                        "home_task_state": 9,
                        "bed1_task_state": 3,
                        "bed3_task_state": 6,
                        "max_speed_m_s": 3.0,
                        # With vmax=3.0 m/s, terminal=0.10 m/s at 0.10 m,
                        # keep full speed until 1.00 m, then follow a continuous
                        # home-only braking envelope. This preserves roughly
                        # the previous 4.99 m/s^2 equivalent deceleration.
                        "soft_decel_m_s2": 4.9875,
                        "terminal_speed_m_s": 0.10,
                        "terminal_distance_m": 0.10,
                        "decel_start_distance_m": 1.00,
                        # Bed approaches use independent map-X/map-Y envelopes.
                        # The measured wheel velocity reserves 220 ms of
                        # response distance before the physical braking term.
                        "bed_max_speed_m_s": 3.0,
                        "bed_forward_decel_m_s2": 2.0,
                        "bed_side_decel_m_s2": 2.2,
                        "bed_reaction_time_s": 0.22,
                        "bed_braking_margin_m": 0.03,
                        "pose_timeout_s": 0.5,
                        "wheel_odom_timeout_s": 0.15,
                    }
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
            # Keep the field-tested collision profile in one monitor. The
            # home/bed limiter runs first, while ApproachPolygon and the final
            # StopPolygon share the same pointcloud snapshot and output path.
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[params_file],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_safety_pipeline",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": autostart,
                        "node_names": ["collision_monitor"],
                    }
                ],
            ),
            Node(
                package="obstacle_detector",
                executable="medical_navigator",
                name="medical_navigator",
                output="screen",
                parameters=[
                    {
                        "map_config": field_map,
                        "map_frame": "map",
                        "yaw_sign": -1.0,
                        "goal_handoff_timeout_s": 1.0,
                    }
                ],
            ),
        ]
    )
