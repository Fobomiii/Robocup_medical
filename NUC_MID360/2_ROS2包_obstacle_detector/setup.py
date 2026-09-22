from setuptools import setup
import os
from glob import glob

package_name = "obstacle_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "stm32_bridge = obstacle_detector.stm32_bridge:main",
            "medical_navigator = obstacle_detector.medical_navigator:main",
            "lidar_self_filter = obstacle_detector.lidar_transform:main",
            "lidar_odometry_guard = obstacle_detector.lidar_odometry_guard:main",
            "code_scanner = obstacle_detector.code_scanner:main",
        ],
    },
)
