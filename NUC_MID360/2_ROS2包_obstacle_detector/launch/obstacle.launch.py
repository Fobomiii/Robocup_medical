import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
                        # Chassis radius 0.225 m plus 35 mm for frame/wheel returns.
                        "self_radius": 0.26,
                        "self_min_z": -0.05,
                        # The upright lidar mount/aluminium returns reach about
                        # z=0.54 m and are part of the robot, not obstacles.
                        "self_max_z": 0.65,
                        # Reject the slightly tilted floor plane before Nav2
                        # inflates individual floor returns into false walls.
                        "ground_filter_enabled": True,
                        "ground_distance_threshold": 0.04,
                        # Match the existing 8 cm Nav2 minimum height, but
                        # measure it from the fitted floor rather than base z.
                        "ground_removal_below": 0.05,
                        "ground_removal_above": 0.08,
                        "ground_max_tilt_deg": 12.0,
                        "ground_max_origin_height": 0.15,
                        "ground_min_inliers": 300,
                        "ground_candidate_min_z": -0.25,
                        "ground_candidate_max_z": 0.30,
                        "ground_min_radius": 0.30,
                        "ground_max_radius": 5.0,
                        "ground_ransac_iterations": 40,
                        "reject_livox_noise": True,
                        "livox_noise_mask": 15,
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
                        # NUC-side linear clamp: 1.00 m/s per body-axis
                        # component. STM32 keeps a separate 2.00 m/s cap.
                        "max_speed_mm_s": 1000.0,
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "enforce_task_gate": ParameterValue(
                            enforce_task_gate, value_type=bool
                        ),
                        "field_config": field_map,
                    }
                ],
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
            ),
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
                name="lifecycle_manager_collision_monitor",
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
                    }
                ],
            ),
        ]
    )
