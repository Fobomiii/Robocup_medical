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

    # YAML cannot expand an ament package path itself. Rewrite only this leaf
    # parameter so bt_navigator receives the installed XML's absolute path.
    nav2_params_file = RewrittenYaml(
        source_file=params_file,
        param_rewrites={"default_bt_xml_filename": bt_xml_file},
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
                        # then keep only 10 mm beyond the measured 0.155 m base.
                        "cone_height": 0.65,
                        "physical_base_radius": 0.155,
                        "base_radius": 0.165,
                        "min_z": 0.08,
                        # Reject compact clusters that never reach the ground.
                        # The mounted arm sits at 0.64-0.70 m and fits the
                        # cone cluster test, so without this it gets a false
                        # base disk at z=0.12 m in front of the robot.
                        "min_base_z": 0.35,
                        # Detect and expand competition cones early enough for
                        # the 2 Hz global replanner at the 2.0 m/s speed limit.
                        "max_range": 6.0,
                        # Hold only synthetic cone-base disks briefly. The
                        # source cloud remains frame-by-frame so people do
                        # not leave persistence trails.
                        "persistence_s": 0.35,
                        "persistence_frame": "odom",
                        "persistence_match_distance": 0.40,
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
                        # Limit task-gate release and re-acceleration after a
                        # collision slowdown. Braking still passes immediately.
                        "gate_release_wheel_accel_m_s2": 2.5,
                        "gate_release_yaw_radius_m": 0.25,
                        "gate_release_rearm_drop_m_s": 0.25,
                        "nurse_scan_stop_linear_m_s": 0.03,
                        "nurse_scan_stop_angular_rad_s": 0.05,
                        "nurse_scan_stop_settle_s": 0.15,
                        "wheel_odom_timeout_s": 0.15,
                        "ekf_ops_position_std_m": 0.02,
                        "ekf_hwt_yaw_std_rad": 0.015,
                        "ekf_wheel_forward_std_m_s": 0.08,
                        "ekf_wheel_lateral_std_m_s": 0.16,
                        "ekf_wheel_yaw_std_rad_s": 0.12,
                        "ekf_linear_accel_std_m_s2": 1.5,
                        "ekf_yaw_accel_std_rad_s2": 1.5,
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
                        # NUC-side linear clamp: 2.00 m/s per body-axis
                        # component. STM32 keeps a separate 4.00 m/s cap.
                        "max_speed_mm_s": 2000.0,
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
                executable="home_approach_limiter",
                name="home_approach_limiter",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/cmd_vel",
                        "output_topic": "/cmd_vel_home_limited",
                        "pose_topic": "/medical_nav/robot_pose",
                        "task_state_topic": "/medical_nav/task_state",
                        "field_config": field_map,
                        "home_task_state": 9,
                        "bed1_task_state": 3,
                        "bed3_task_state": 6,
                        "max_speed_m_s": 2.0,
                        # With vmax=2.0 m/s, terminal=0.10 m/s at 0.10 m,
                        # this begins the home-only envelope at exactly 1.50 m.
                        "soft_decel_m_s2": 1.425,
                        "terminal_speed_m_s": 0.10,
                        "terminal_distance_m": 0.10,
                        # Bed approaches retain full speed in open space, then
                        # cap planar vx/vy together before Nav2 hands control
                        # to the STM32 laser docking correction.
                        "bed_max_speed_m_s": 2.0,
                        "bed_soft_decel_m_s2": 1.0,
                        "bed_terminal_speed_m_s": 0.15,
                        "bed_terminal_distance_m": 0.20,
                        "pose_timeout_s": 0.5,
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
