"""
Livox Mid360 ROS2 Launch 文件
xfer_format = 0  →  发布 PointCloud2 到话题 /livox/lidar
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# ── 用户可调参数 ────────────────────────────────────────────────
xfer_format     = 0       # 0=PointCloud2（/livox/lidar），1=CustomMsg
multi_topic     = 0       # 0=所有雷达共用同一话题
data_src        = 0       # 0=雷达实时数据
publish_freq    = 10.0    # 发布频率 Hz（5/10/20/50）
output_type     = 0
frame_id        = 'livox_frame'
lvx_file_path   = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'
# ───────────────────────────────────────────────────────────────

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=livox_ros2_params
    )
    return LaunchDescription([livox_driver])
