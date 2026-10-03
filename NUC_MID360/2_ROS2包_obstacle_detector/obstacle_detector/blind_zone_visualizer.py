#!/usr/bin/env python3
"""Visualize the chassis X-shaped lidar shadows and current motion risk."""

from __future__ import annotations

import json
import math
import time

from geometry_msgs.msg import Point, Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _angle_error(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


class BlindZoneVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("blind_zone_visualizer")
        self.frame_id = str(self.declare_parameter("frame_id", "base_link").value)
        self.marker_topic = str(
            self.declare_parameter(
                "marker_topic", "/medical_nav/blind_zones"
            ).value
        )
        self.status_topic = str(
            self.declare_parameter(
                "status_topic", "/medical_nav/blind_zone_status"
            ).value
        )
        self.cmd_vel_topic = str(
            self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        )
        self.angles_rad = tuple(
            math.radians(float(value))
            for value in self.declare_parameter(
                "angles_deg", [45.0, 135.0, -135.0, -45.0]
            ).value
        )
        self.origin_x_m = float(
            self.declare_parameter("origin_x_m", 0.0).value
        )
        self.origin_y_m = float(
            self.declare_parameter("origin_y_m", 0.0).value
        )
        self.start_distance_m = float(
            self.declare_parameter("start_distance_m", 0.23).value
        )
        self.width_m = float(self.declare_parameter("width_m", 0.18).value)
        self.min_length_m = float(
            self.declare_parameter("min_length_m", 0.80).value
        )
        self.max_length_m = float(
            self.declare_parameter("max_length_m", 2.50).value
        )
        self.braking_decel_m_s2 = float(
            self.declare_parameter("braking_decel_m_s2", 1.30).value
        )
        self.reaction_time_s = float(
            self.declare_parameter("reaction_time_s", 0.25).value
        )
        self.safety_margin_m = float(
            self.declare_parameter("safety_margin_m", 0.25).value
        )
        self.half_width_rad = math.radians(
            float(self.declare_parameter("half_width_deg", 7.0).value)
        )
        self.cmd_timeout_s = float(
            self.declare_parameter("cmd_timeout_s", 0.50).value
        )
        publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 10.0).value
        )

        if not self.angles_rad:
            raise ValueError("at least one blind-zone angle is required")
        for value in (
            self.start_distance_m,
            self.width_m,
            self.min_length_m,
            self.max_length_m,
            self.braking_decel_m_s2,
            self.reaction_time_s,
            self.safety_margin_m,
            self.half_width_rad,
            self.cmd_timeout_s,
            publish_rate_hz,
        ):
            if value <= 0.0:
                raise ValueError("blind-zone visualization parameters must be positive")
        if self.max_length_m < self.min_length_m:
            raise ValueError("max_length_m must not be below min_length_m")

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.cmd_received_s = 0.0
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel, 10)
        self.create_timer(1.0 / publish_rate_hz, self._publish)
        self.get_logger().info(
            "X-shaped lidar blind-zone visualization ready | "
            f"angles={[round(math.degrees(value), 1) for value in self.angles_rad]} "
            f"width={self.width_m:.2f} m length={self.min_length_m:.2f}-"
            f"{self.max_length_m:.2f} m"
        )

    def _cmd_vel(self, message: Twist) -> None:
        self.linear_x = float(message.linear.x)
        self.linear_y = float(message.linear.y)
        self.cmd_received_s = time.monotonic()

    def _speed_and_heading(self) -> tuple[float, float | None]:
        if time.monotonic() - self.cmd_received_s > self.cmd_timeout_s:
            return 0.0, None
        speed = math.hypot(self.linear_x, self.linear_y)
        if speed <= 1.0e-4:
            return speed, None
        return speed, math.atan2(self.linear_y, self.linear_x)

    def _length(self, speed_m_s: float) -> float:
        braking_distance = speed_m_s * speed_m_s / (
            2.0 * self.braking_decel_m_s2
        )
        requested = (
            braking_distance
            + speed_m_s * self.reaction_time_s
            + self.safety_margin_m
        )
        return min(self.max_length_m, max(self.min_length_m, requested))

    @staticmethod
    def _quaternion(marker: Marker, yaw: float) -> None:
        marker.pose.orientation.z = math.sin(0.5 * yaw)
        marker.pose.orientation.w = math.cos(0.5 * yaw)

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        speed, motion_heading = self._speed_and_heading()
        length = self._length(speed)
        nearest_angle = None
        nearest_error = None
        if motion_heading is not None:
            nearest_angle = min(
                self.angles_rad,
                key=lambda angle: _angle_error(motion_heading, angle),
            )
            nearest_error = _angle_error(motion_heading, nearest_angle)
        severe_overlap = (
            nearest_error is not None and nearest_error <= self.half_width_rad
        )

        markers = MarkerArray()
        for index, angle in enumerate(self.angles_rad):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "lidar_blind_strip"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            center_distance = self.start_distance_m + 0.5 * length
            marker.pose.position.x = self.origin_x_m + center_distance * math.cos(angle)
            marker.pose.position.y = self.origin_y_m + center_distance * math.sin(angle)
            marker.pose.position.z = 0.04
            self._quaternion(marker, angle)
            marker.scale.x = length
            marker.scale.y = self.width_m
            marker.scale.z = 0.06
            is_active = (
                motion_heading is not None
                and _angle_error(motion_heading, angle) <= self.half_width_rad
            )
            marker.color.r = 1.0
            marker.color.g = 0.08 if is_active else 0.25
            marker.color.b = 0.02
            marker.color.a = 0.72 if is_active else 0.32
            marker.frame_locked = True
            markers.markers.append(marker)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "lidar_blind_label"
            label.id = 100 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = self.origin_x_m + (
                self.start_distance_m + length
            ) * math.cos(angle)
            label.pose.position.y = self.origin_y_m + (
                self.start_distance_m + length
            ) * math.sin(angle)
            label.pose.position.z = 0.16
            label.pose.orientation.w = 1.0
            label.scale.z = 0.14
            label.color.r = 1.0
            label.color.g = 0.8 if not is_active else 0.2
            label.color.b = 0.2
            label.color.a = 0.95
            label.text = f"blind {math.degrees(angle):.0f} deg"
            label.frame_locked = True
            markers.markers.append(label)

        chassis = Marker()
        chassis.header.frame_id = self.frame_id
        chassis.header.stamp = stamp
        chassis.ns = "lidar_blind_chassis"
        chassis.id = 200
        chassis.type = Marker.CYLINDER
        chassis.action = Marker.ADD
        chassis.pose.orientation.w = 1.0
        chassis.pose.position.z = 0.025
        chassis.scale.x = 0.46
        chassis.scale.y = 0.46
        chassis.scale.z = 0.05
        chassis.color.r = 0.35
        chassis.color.g = 0.38
        chassis.color.b = 0.42
        chassis.color.a = 0.50
        chassis.frame_locked = True
        markers.markers.append(chassis)

        if motion_heading is not None:
            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = stamp
            arrow.ns = "lidar_blind_motion"
            arrow.id = 201
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=0.0, y=0.0, z=0.12),
                Point(
                    x=0.75 * math.cos(motion_heading),
                    y=0.75 * math.sin(motion_heading),
                    z=0.12,
                ),
            ]
            arrow.scale.x = 0.05
            arrow.scale.y = 0.10
            arrow.scale.z = 0.12
            arrow.color.r = 1.0 if severe_overlap else 0.1
            arrow.color.g = 0.15 if severe_overlap else 1.0
            arrow.color.b = 0.05
            arrow.color.a = 1.0
            arrow.frame_locked = True
            markers.markers.append(arrow)

        self.marker_pub.publish(markers)
        status = String()
        status.data = json.dumps(
            {
                "speed_m_s": round(speed, 3),
                "blind_length_m": round(length, 3),
                "motion_heading_deg": (
                    None
                    if motion_heading is None
                    else round(math.degrees(motion_heading), 1)
                ),
                "nearest_blind_angle_deg": (
                    None
                    if nearest_angle is None
                    else round(math.degrees(nearest_angle), 1)
                ),
                "nearest_blind_error_deg": (
                    None
                    if nearest_error is None
                    else round(math.degrees(nearest_error), 1)
                ),
                "severe_overlap": severe_overlap,
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlindZoneVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
