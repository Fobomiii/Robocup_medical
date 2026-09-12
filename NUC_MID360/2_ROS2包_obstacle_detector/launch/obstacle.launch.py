from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("obstacle_detector"),
        "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate",   default_value="115200"),
        Node(
            package="obstacle_detector",
            executable="obstacle_node",
            name="obstacle_detector",
            output="screen",
            parameters=[cfg, {
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate":   LaunchConfiguration("baud_rate"),
            }],
        ),
    ])
