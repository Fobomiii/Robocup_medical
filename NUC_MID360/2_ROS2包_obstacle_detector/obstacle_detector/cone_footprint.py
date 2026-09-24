#!/usr/bin/env python3
"""Expand compact cone returns into their known ground footprint."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, String

from obstacle_detector.cone_footprint_core import expand_cone_footprints
from obstacle_detector.lidar_transform import xyz_from_cloud


class ConeFootprintCompensator(Node):
    def __init__(self) -> None:
        super().__init__("cone_footprint_compensator")
        self.declare_parameter("input_topic", "/livox/lidar_filtered")
        self.declare_parameter("output_topic", "/livox/lidar_nav")
        self.declare_parameter("base_radius", 0.155)
        self.declare_parameter("cone_height", 0.65)
        self.declare_parameter("min_z", 0.08)
        self.declare_parameter("min_range", 0.30)
        self.declare_parameter("max_range", 4.5)
        self.declare_parameter("cluster_cell", 0.08)
        self.declare_parameter("min_points", 3)
        self.declare_parameter("max_span", 0.32)
        self.declare_parameter("min_vertical_span", 0.06)
        self.declare_parameter("disk_spacing", 0.05)
        self.declare_parameter("disk_height", 0.12)

        get = lambda name: self.get_parameter(name).value
        self.input_topic = str(get("input_topic"))
        self.output_topic = str(get("output_topic"))
        self.parameters = {
            "base_radius": float(get("base_radius")),
            "cone_height": float(get("cone_height")),
            "min_z": float(get("min_z")),
            "min_range": float(get("min_range")),
            "max_range": float(get("max_range")),
            "cluster_cell": float(get("cluster_cell")),
            "min_points": int(get("min_points")),
            "max_span": float(get("max_span")),
            "min_vertical_span": float(get("min_vertical_span")),
            "disk_spacing": float(get("disk_spacing")),
            "disk_height": float(get("disk_height")),
        }
        if self.parameters["base_radius"] <= 0.0:
            raise ValueError("base_radius must be positive")
        if self.parameters["cone_height"] <= self.parameters["min_z"]:
            raise ValueError("cone_height must exceed min_z")
        if self.parameters["cluster_cell"] <= 0.0:
            raise ValueError("cluster_cell must be positive")
        if self.parameters["disk_spacing"] <= 0.0:
            raise ValueError("disk_spacing must be positive")

        self.cloud_count = 0
        self.detected_count = 0
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.status_publisher = self.create_publisher(
            String, "/medical_nav/cone_compensation_status", 10
        )
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(2.0, self._publish_status)
        self.get_logger().info(
            f"Cone footprint {self.input_topic} -> {self.output_topic}: "
            f"height={self.parameters['cone_height']:.3f} m, "
            f"base_radius={self.parameters['base_radius']:.3f} m"
        )

    def _cloud_callback(self, message: PointCloud2) -> None:
        xyz = xyz_from_cloud(message)
        expanded, centers = expand_cone_footprints(xyz, **self.parameters)
        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = message.header.frame_id
        self.publisher.publish(pc2.create_cloud_xyz32(header, expanded))
        self.cloud_count += 1
        self.detected_count = len(centers)

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "clouds": self.cloud_count,
                "cones": self.detected_count,
                "base_radius_m": self.parameters["base_radius"],
                "cone_height_m": self.parameters["cone_height"],
            }
        )
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ConeFootprintCompensator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
