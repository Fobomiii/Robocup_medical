#!/usr/bin/env python3
"""Publish a non-controlling physical braking-envelope shadow command."""

from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String

from .braking_safety_core import evaluate_braking_safety
from .qos_profiles import low_latency_sensor_qos


class BrakingSafetyShadow(Node):
    """Evaluate braking safety while leaving the live command chain untouched."""

    def __init__(self) -> None:
        super().__init__("braking_safety_shadow")
        self.input_topic = str(
            self.declare_parameter("input_topic", "/cmd_vel_home_limited").value
        )
        self.cloud_topic = str(
            self.declare_parameter("cloud_topic", "/livox/lidar_safety").value
        )
        self.odom_topic = str(self.declare_parameter("odom_topic", "/odom").value)
        self.output_topic = str(
            self.declare_parameter(
                "output_topic", "/medical_nav/braking_shadow_cmd_vel"
            ).value
        )
        self.status_topic = str(
            self.declare_parameter(
                "status_topic", "/medical_nav/braking_shadow_status"
            ).value
        )
        self.braking_decel_m_s2 = float(
            self.declare_parameter("model_braking_decel_m_s2", 2.5).value
        )
        self.reaction_time_s = float(
            self.declare_parameter("reaction_time_s", 0.20).value
        )
        self.safety_margin_m = float(
            self.declare_parameter("safety_margin_m", 0.15).value
        )
        self.footprint_radius_m = float(
            self.declare_parameter("footprint_radius_m", 0.245).value
        )
        self.max_speed_m_s = float(
            self.declare_parameter("max_speed_m_s", 3.0).value
        )
        self.min_points = int(self.declare_parameter("min_points", 5).value)
        self.max_lookahead_m = float(
            self.declare_parameter("max_lookahead_m", 4.0).value
        )
        self.command_timeout_s = float(
            self.declare_parameter("command_timeout_s", 0.30).value
        )
        self.cloud_timeout_s = float(
            self.declare_parameter("cloud_timeout_s", 0.20).value
        )
        self.odom_timeout_s = float(
            self.declare_parameter("odom_timeout_s", 0.15).value
        )
        self.update_rate_hz = float(
            self.declare_parameter("update_rate_hz", 50.0).value
        )

        if self.update_rate_hz <= 0.0:
            raise ValueError("update rate must be positive")
        self.latest_command: Twist | None = None
        self.latest_command_s: float | None = None
        self.latest_points: np.ndarray | None = None
        self.latest_cloud_s: float | None = None
        self.latest_cloud_frame = ""
        self.latest_cloud_stamp_ns = 0
        self.latest_odom: Odometry | None = None
        self.latest_odom_s: float | None = None

        normal_qos = QoSProfile(depth=10)
        self.command_subscription = self.create_subscription(
            Twist, self.input_topic, self._command_callback, normal_qos
        )
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self._cloud_callback,
            low_latency_sensor_qos(),
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            low_latency_sensor_qos(),
        )
        self.command_publisher = self.create_publisher(
            Twist, self.output_topic, normal_qos
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, normal_qos
        )
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self._update)
        self.get_logger().info(
            "Braking safety shadow is observation-only; live control remains unchanged"
        )

    def _command_callback(self, message: Twist) -> None:
        self.latest_command = message
        self.latest_command_s = time.monotonic()

    def _cloud_callback(self, message: PointCloud2) -> None:
        self.latest_cloud_s = time.monotonic()
        self.latest_cloud_frame = message.header.frame_id.lstrip("/")
        self.latest_cloud_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        points = pc2.read_points(
            message, field_names=("x", "y"), skip_nans=True
        )
        if points.size == 0:
            self.latest_points = np.empty((0, 2), dtype=np.float64)
            return
        self.latest_points = np.column_stack((points["x"], points["y"])).astype(
            np.float64, copy=False
        )

    def _odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        self.latest_odom_s = time.monotonic()

    @staticmethod
    def _scaled_command(message: Twist, scale: float) -> Twist:
        output = Twist()
        output.linear.x = message.linear.x * scale
        output.linear.y = message.linear.y * scale
        output.linear.z = message.linear.z
        output.angular.x = message.angular.x
        output.angular.y = message.angular.y
        output.angular.z = message.angular.z
        return output

    def _publish_invalid(self, reason: str, now_s: float) -> None:
        self.command_publisher.publish(Twist())
        status = {
            "shadow_mode": True,
            "affects_control": False,
            "valid": False,
            "reason": reason,
            "command_age_s": None
            if self.latest_command_s is None
            else round(now_s - self.latest_command_s, 4),
            "cloud_age_s": None
            if self.latest_cloud_s is None
            else round(now_s - self.latest_cloud_s, 4),
            "odom_age_s": None
            if self.latest_odom_s is None
            else round(now_s - self.latest_odom_s, 4),
            "cloud_frame": self.latest_cloud_frame,
            "cloud_stamp_ns": self.latest_cloud_stamp_ns,
        }
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self.status_publisher.publish(message)

    def _update(self) -> None:
        now_s = time.monotonic()
        if self.latest_command is None or self.latest_command_s is None:
            self._publish_invalid("command_missing", now_s)
            return
        if now_s - self.latest_command_s > self.command_timeout_s:
            self._publish_invalid("command_stale", now_s)
            return
        if self.latest_points is None or self.latest_cloud_s is None:
            self._publish_invalid("cloud_missing", now_s)
            return
        if len(self.latest_points) == 0:
            self._publish_invalid("cloud_empty", now_s)
            return
        if self.latest_cloud_frame != "base_link":
            self._publish_invalid("cloud_not_in_base_link", now_s)
            return
        if now_s - self.latest_cloud_s > self.cloud_timeout_s:
            self._publish_invalid("cloud_stale", now_s)
            return
        if self.latest_odom is None or self.latest_odom_s is None:
            self._publish_invalid("odom_missing", now_s)
            return
        if now_s - self.latest_odom_s > self.odom_timeout_s:
            self._publish_invalid("odom_stale", now_s)
            return

        command = self.latest_command
        measured = self.latest_odom.twist.twist
        try:
            decision = evaluate_braking_safety(
                self.latest_points,
                float(command.linear.x),
                float(command.linear.y),
                float(measured.linear.x),
                float(measured.linear.y),
                braking_decel_m_s2=self.braking_decel_m_s2,
                reaction_time_s=self.reaction_time_s,
                safety_margin_m=self.safety_margin_m,
                footprint_radius_m=self.footprint_radius_m,
                max_speed_m_s=self.max_speed_m_s,
                min_points=self.min_points,
                max_lookahead_m=self.max_lookahead_m,
            )
        except ValueError as error:
            self._publish_invalid(f"invalid_data:{error}", now_s)
            return
        self.command_publisher.publish(self._scaled_command(command, decision.scale))
        status = {
            "shadow_mode": True,
            "affects_control": False,
            "valid": True,
            "reason": decision.reason,
            "collision_distance_m": decision.collision_distance_m,
            "effective_distance_m": decision.effective_distance_m,
            "command_speed_m_s": decision.command_speed_m_s,
            "measured_speed_m_s": decision.measured_speed_m_s,
            "stopping_distance_m": decision.stopping_distance_m,
            "speed_limit_m_s": decision.speed_limit_m_s,
            "shadow_speed_m_s": decision.shadow_speed_m_s,
            "scale": decision.scale,
            "candidate_point_count": decision.candidate_point_count,
            "would_limit": decision.would_limit,
            "would_stop": decision.would_stop,
            "command_age_s": round(now_s - self.latest_command_s, 4),
            "cloud_age_s": round(now_s - self.latest_cloud_s, 4),
            "odom_age_s": round(now_s - self.latest_odom_s, 4),
            "cloud_frame": self.latest_cloud_frame,
            "cloud_stamp_ns": self.latest_cloud_stamp_ns,
            "model_braking_decel_m_s2": self.braking_decel_m_s2,
            "reaction_time_s": self.reaction_time_s,
            "safety_margin_m": self.safety_margin_m,
            "footprint_radius_m": self.footprint_radius_m,
        }
        status_message = String()
        status_message.data = json.dumps(status, separators=(",", ":"))
        self.status_publisher.publish(status_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BrakingSafetyShadow()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
