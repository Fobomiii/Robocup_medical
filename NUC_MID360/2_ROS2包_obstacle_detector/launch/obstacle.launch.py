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
            # Formal workflow: nurse QR and bedside CODE128 scans are required.
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
                        # Remove chassis returns through 0.245 m while leaving
                        # the surrounding collision-monitor band observable.
                        "self_radius": 0.245,
                        "self_filter_shape": "polygon",
                        "self_polygon_sides": 16,
                        "self_polygon_rotation_deg": 0.0,
                        "self_polygon_topic": "/medical_nav/safety_self_filter_polygon",
                        "calibration_stop_radius": 0.285,
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
                        "arm_yaw_deg": 0.0,
                        "arm_min_z": 0.45,
                        "arm_max_z": 0.82,
                        "ground_filter_enabled": False,
                        # Ground fitting is disabled on the emergency path,
                        # but the node still validates this radius against
                        # self_radius during startup. Keep the unused value
                        # outside the 0.245 m safety self-filter.
                        "ground_removal_min_radius": 0.27,
                        "reject_livox_noise": True,
                        "livox_noise_mask": 15,
                        "gate_on_stm32_ready": True,
                        "motion_compensation_enabled": True,
                        "odom_topic": "/odom",
                        "motion_history_s": 2.0,
                        "max_motion_extrapolation_s": 0.06,
                        "max_cloud_span_s": 0.20,
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
                        # Chassis radius 0.23 m plus 15 mm for frame/wheel returns.
                        "self_radius": 0.245,
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
                        # The mounted arm overhangs the 0.245 m self cylinder,
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
                        "arm_yaw_deg": 0.0,
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
                        # the accepted plane outside the self-filter edge.
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
                        "motion_compensation_enabled": True,
                        "odom_topic": "/odom",
                        "motion_history_s": 2.0,
                        "max_motion_extrapolation_s": 0.06,
                        "max_cloud_span_s": 0.20,
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
                        # Real competition cones produce many returns at the
                        # configured 10 Hz cloud rate. Reject sparse bed/wall
                        # fragments before creating a synthetic base disk.
                        "min_points": 15,
                        "min_vertical_span": 0.12,
                        "center_merge_distance": 0.28,
                        # Reject compact clusters that never reach the ground.
                        # The mounted arm sits at 0.64-0.70 m and fits the
                        # cone cluster test, so without this it gets a false
                        # base disk at z=0.12 m in front of the robot.
                        "min_base_z": 0.35,
                        # Detect and expand competition cones up to two seconds
                        # ahead at the validated 3.0 m/s translation limit.
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
                executable="stm32_bridge",
                name="stm32_bridge",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "baud_rate": baud_rate,
                        "yaw_sign": -1.0,
                        "cmd_vel_topic": "/cmd_vel_safe",
                        "collision_input_topic": "/cmd_vel_home_limited",
                        # No fixed start/handoff delay. A fresh matching
                        # FOLLOWING status and the task gate remain required.
                        "goal_handoff_hold_s": 0.0,
                        "navigator_following_hold_s": 0.0,
                        "navigator_status_timeout_s": 1.2,
                        # The STM32 is intentionally hard-reset between runs
                        # after the unpowered chassis has been pushed home.
                        # Require fresh stationary telemetry before reopening
                        # motion and the lidar navigation pipeline.
                        "stm32_telemetry_timeout_s": 0.25,
                        "stm32_recovery_hold_s": 0.75,
                        "stm32_recovery_max_linear_m_s": 0.03,
                        "stm32_recovery_max_angular_rad_s": 0.05,
                        # Zero disables the downstream acceleration ramp;
                        # Nav2's CLOSED_LOOP smoother owns navigation ramps.
                        "gate_release_wheel_accel_m_s2": 0.0,
                        "gate_release_yaw_radius_m": 0.25,
                        # Retained for deployed-parameter compatibility; the
                        # continuous limiter no longer needs re-arming.
                        "gate_release_rearm_drop_m_s": 0.25,
                        # Physical cross drive: X and Y are each capped at
                        # 3.0 m/s. Diagonal commands are not normalized.
                        "max_axis_speed_m_s": 3.0,
                        # DWB supplies the unified planar command.  Bed/home
                        # travel keeps the surveyed 0-degree heading.  Nurse
                        # scan is translation-only in this test: the bridge
                        # forces wz=0 for TASK_NAV_NURSE, including fallback
                        # viewpoints.
                        "heading_correction_max_wz_rad_s": 0.30,
                        "active_rotation_max_wz_rad_s": 0.80,
                        "active_rotation_linear_threshold_m_s": 0.05,
                        # Navigation yaw hold runs on STM32. Keep the bridge
                        # command/TF/odometry path unchanged.
                        "heading_hold_enabled": False,
                        "heading_hold_task_states": [1, 3, 6, 9],
                        "heading_hold_target_deg": 0.0,
                        "heading_hold_kp": 2.2,
                        "heading_hold_ki": 0.0,
                        "heading_hold_kd": 0.45,
                        "heading_hold_deadband_deg": 1.0,
                        "heading_hold_max_wz_rad_s": 0.40,
                        "heading_hold_integral_limit_rad_s": 0.05,
                        "heading_hold_yaw_rate_deadband_rad_s": 0.02,
                        "nurse_scan_stop_linear_m_s": 0.03,
                        "nurse_scan_stop_angular_rad_s": 0.05,
                        "nurse_scan_stop_settle_s": 0.15,
                        "wheel_odom_timeout_s": 0.15,
                        # Covariance metadata only; bridge publishes direct
                        # OPS/HWT pose and measured wheel twist without EKF.
                        "ops_position_std_m": 0.02,
                        "wheel_forward_std_m_s": 0.08,
                        "wheel_lateral_std_m_s": 0.16,
                        "wheel_yaw_std_rad_s": 0.12,
                        # Pre-generated bedside announcements at maximum
                        # ALSA mixer volume.
                        "tts_bed1_audio_file": "/home/fzurobot/Downloads/1_wav.wav",
                        "tts_bed3_audio_file": "/home/fzurobot/Downloads/3_wav.wav",
                        "tts_volume_percent": 100,
                        "tts_audio_device": "pulse",
                        # Keep a silent Pulse stream open while this service is
                        # running. Pulse still mixes calls and other desktop
                        # audio, while the amplifier no longer clips speech.
                        "tts_keepalive_enabled": True,
                        "tts_lead_silence_s": 0.0,
                        "tts_tail_silence_s": 0.0,
                        "tts_timeout_s": 8.0,
                        # Per-axis protocol guard after cross-drive limiting.
                        # STM32 repeats the same hard safety caps.
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
                        "bond_timeout": 10.0,
                        "attempt_respawn_reconnection": True,
                        "bond_respawn_max_duration": 30.0,
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
                        # Preserve 3.0 m/s on straights, then apply a braking
                        # envelope before real path corners so the 20 kg
                        # chassis can redirect without overshoot.
                        "enabled": True,
                        "raw_path_topic": "/plan",
                        "smoothed_path_topic": "/transformed_global_plan",
                        "pose_topic": "/medical_nav/robot_pose",
                        "output_topic": "/speed_limit",
                        "status_topic": "/medical_nav/corner_speed_status",
                        # Shadow-only Menger-curvature reference profile. These
                        # topics are recorded for tuning and are not consumed by
                        # Nav2, the speed limiter, or the chassis command chain.
                        "profile_shadow_enabled": True,
                        "reference_profile_topic": "/medical_nav/reference_speed_profile",
                        "reference_status_topic": "/medical_nav/reference_speed_status",
                        "profile_odom_topic": "/odom",
                        "profile_max_speed_m_s": 3.0,
                        "profile_min_curve_speed_m_s": 0.50,
                        "profile_lateral_accel_m_s2": 2.30,
                        "profile_accel_m_s2": 4.0,
                        "profile_braking_decel_m_s2": 2.0,
                        "profile_reaction_time_s": 0.20,
                        "profile_braking_margin_m": 0.20,
                        "profile_lookahead_distance_m": 6.0,
                        "profile_sample_step_m": 0.10,
                        "profile_curvature_span_m": 0.50,
                        "profile_curvature_filter_window_m": 0.20,
                        "profile_command_lookahead_m": 0.25,
                        "profile_odom_timeout_s": 0.50,
                        "profile_publish_rate_hz": 5.0,
                        "max_speed_m_s": 3.0,
                        # Ordinary corners are capped at 1.0 m/s. Curvature may
                        # lower the target further, down to this crawl floor.
                        "min_corner_speed_m_s": 0.50,
                        "max_corner_speed_m_s": 1.00,
                        "lateral_accel_m_s2": 2.30,
                        "braking_decel_m_s2": 2.00,
                        "reaction_time_s": 0.20,
                        "release_hold_s": 0.50,
                        "straight_confirmation_s": 0.20,
                        "lookahead_distance_m": 3.50,
                        # Average over 0.75 m on each side so short replanning
                        # kinks do not appear as persistent route corners.
                        "tangent_span_m": 0.75,
                        "sample_step_m": 0.10,
                        "min_turn_angle_deg": 35.0,
                        "braking_margin_m": 0.20,
                        "path_timeout_s": 1.50,
                        "pose_timeout_s": 0.50,
                        "smoothing_wait_s": 0.25,
                        "update_rate_hz": 20.0,
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
                        "home_status_topic": "/medical_nav/home_approach_status",
                        "field_status_topic": "/medical_nav/field_boundary_status",
                        # Keep this relay in the command chain, but disable the
                        # former field/bed/Home velocity envelopes. Collision
                        # Monitor remains the final active safety layer.
                        "enabled": False,
                        "field_config": field_map,
                        "home_task_state": 9,
                        "bed1_task_state": 3,
                        "bed3_task_state": 6,
                        "max_speed_m_s": 3.0,
                        # Physical field bounds in ROS map coordinates. The
                        # centre-space guard insets these by the full chassis
                        # projection plus localisation uncertainty.
                        "field_x_min_m": -0.5,
                        "field_x_max_m": 6.5,
                        "field_y_min_m": -3.5,
                        "field_y_max_m": 3.5,
                        "field_footprint_radius_m": 0.24,
                        "field_localization_margin_m": 0.05,
                        "field_braking_decel_m_s2": 2.0,
                        "field_reaction_time_s": 0.22,
                        "field_braking_margin_m": 0.03,
                        # Home pre-braking is derived from measured velocity:
                        # v*t_reaction + v^2/(2*a) + margin. Keep the cross-axis
                        # free for centring instead of scaling the whole vector.
                        "home_braking_decel_m_s2": 2.0,
                        "home_reaction_time_s": 0.22,
                        "home_braking_margin_m": 0.03,
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
            # Observation-only physical braking envelope. It reads the same
            # pre-collision command and safety cloud as Collision Monitor, but
            # publishes only separate shadow topics and cannot move the robot.
            Node(
                package="obstacle_detector",
                executable="braking_safety_shadow",
                name="braking_safety_shadow",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/cmd_vel_home_limited",
                        "cloud_topic": "/livox/lidar_safety",
                        "odom_topic": "/odom",
                        "output_topic": "/medical_nav/braking_shadow_cmd_vel",
                        "status_topic": "/medical_nav/braking_shadow_status",
                        "model_braking_decel_m_s2": 2.5,
                        "reaction_time_s": 0.20,
                        "safety_margin_m": 0.15,
                        "footprint_radius_m": 0.245,
                        "max_speed_m_s": 3.0,
                        "min_points": 5,
                        "max_lookahead_m": 4.0,
                        "command_timeout_s": 0.30,
                        "cloud_timeout_s": 0.20,
                        "odom_timeout_s": 0.15,
                        "update_rate_hz": 50.0,
                    }
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
            # Keep the field-tested collision profile in one monitor. The
            # approach node above is now a transparent relay, while
            # ApproachPolygon and StopPolygon remain active on the final path.
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
                        "bond_timeout": 10.0,
                        "attempt_respawn_reconnection": True,
                        "bond_respawn_max_duration": 30.0,
                    }
                ],
            ),
            Node(
                package="obstacle_detector",
                executable="medical_navigator",
                name="medical_navigator",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "map_config": field_map,
                        "nav_bt_xml": bt_xml_file,
                        "map_frame": "map",
                        "yaw_sign": -1.0,
                        "goal_handoff_timeout_s": 1.0,
                    }
                ],
            ),
        ]
    )
