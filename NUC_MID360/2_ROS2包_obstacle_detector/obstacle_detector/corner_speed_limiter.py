#!/usr/bin/env python3
"""Publish curvature-aware speed limits upstream of the Nav2 controller."""

from __future__ import annotations

import json
import math
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

from .corner_speed_core import (
    BlindZoneSpeedDecision,
    CornerSpeedDecision,
    Point2D,
    blind_zone_speed_limit,
    corner_speed_limit,
)


class CornerSpeedLimiter(Node):
    """Limit MPPI speed before large translation-direction changes."""

    def __init__(self) -> None:
        super().__init__("corner_speed_limiter")

        self.execution_path_topic = str(
            self.declare_parameter("execution_path_topic", "/plan").value
        )
        self.path_frame_id = str(
            self.declare_parameter("path_frame_id", "map").value
        )
        self.pose_topic = str(
            self.declare_parameter("pose_topic", "/medical_nav/robot_pose").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/speed_limit").value
        )
        self.status_topic = str(
            self.declare_parameter(
                "status_topic", "/medical_nav/corner_speed_status"
            ).value
        )
        self.controller_selector_topic = str(
            self.declare_parameter(
                "controller_selector_topic", "/controller_selector"
            ).value
        )
        self.heading_hold_controller_id = str(
            self.declare_parameter(
                "heading_hold_controller_id", "FollowPathHeadingHold"
            ).value
        )
        self.max_speed_m_s = float(
            self.declare_parameter("max_speed_m_s", 3.0).value
        )
        self.min_corner_speed_m_s = float(
            self.declare_parameter("min_corner_speed_m_s", 1.0).value
        )
        self.lateral_accel_m_s2 = float(
            self.declare_parameter("lateral_accel_m_s2", 1.40).value
        )
        self.heading_hold_min_corner_speed_m_s = float(
            self.declare_parameter(
                "heading_hold_min_corner_speed_m_s", 1.10
            ).value
        )
        self.heading_hold_lateral_accel_m_s2 = float(
            self.declare_parameter(
                "heading_hold_lateral_accel_m_s2", 1.60
            ).value
        )
        self.braking_decel_m_s2 = float(
            self.declare_parameter("braking_decel_m_s2", 1.30).value
        )
        self.lookahead_distance_m = float(
            self.declare_parameter("lookahead_distance_m", 3.50).value
        )
        self.tangent_span_m = float(
            self.declare_parameter("tangent_span_m", 0.35).value
        )
        self.sample_step_m = float(
            self.declare_parameter("sample_step_m", 0.10).value
        )
        self.min_turn_angle_rad = math.radians(
            float(self.declare_parameter("min_turn_angle_deg", 25.0).value)
        )
        self.braking_margin_m = float(
            self.declare_parameter("braking_margin_m", 0.05).value
        )
        self.blind_zone_enabled = bool(
            self.declare_parameter("blind_zone_enabled", True).value
        )
        self.blind_zone_angles_rad = tuple(
            math.radians(float(value))
            for value in self.declare_parameter(
                "blind_zone_angles_deg", [45.0, 135.0, -135.0, -45.0]
            ).value
        )
        self.blind_zone_half_width_rad = math.radians(
            float(self.declare_parameter("blind_zone_half_width_deg", 7.0).value)
        )
        self.blind_zone_heading_tolerance_rad = math.radians(
            float(
                self.declare_parameter(
                    "blind_zone_heading_tolerance_deg", 3.0
                ).value
            )
        )
        self.blind_zone_min_overlap_m = float(
            self.declare_parameter("blind_zone_min_overlap_m", 0.60).value
        )
        self.blind_zone_lookahead_m = float(
            self.declare_parameter("blind_zone_lookahead_m", 2.0).value
        )
        self.blind_zone_effective_half_width_rad = (
            self.blind_zone_half_width_rad
            + self.blind_zone_heading_tolerance_rad
        )
        self.blind_zone_speed_m_s = float(
            self.declare_parameter("blind_zone_speed_m_s", 0.70).value
        )
        self.path_timeout_s = float(
            self.declare_parameter("path_timeout_s", 1.50).value
        )
        self.pose_timeout_s = float(
            self.declare_parameter("pose_timeout_s", 0.50).value
        )
        update_rate_hz = float(
            self.declare_parameter("update_rate_hz", 10.0).value
        )

        self.execution_path: Optional[list[Point2D]] = None
        self.execution_path_received_s = 0.0
        self.execution_path_version: Optional[str] = None
        self.pose_x_m: Optional[float] = None
        self.pose_y_m: Optional[float] = None
        self.pose_yaw_rad: Optional[float] = None
        self.pose_received_s = 0.0
        self.active_controller_id: Optional[str] = None
        self.last_status_s = 0.0

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = QoSReliabilityPolicy.RELIABLE
        selector_qos = QoSProfile(depth=1)
        selector_qos.reliability = QoSReliabilityPolicy.RELIABLE
        selector_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.limit_pub = self.create_publisher(SpeedLimit, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(
            Path, self.execution_path_topic, self._execution_path, path_qos
        )
        self.create_subscription(PoseStamped, self.pose_topic, self._pose, 10)
        self.create_subscription(
            String,
            self.controller_selector_topic,
            self._controller_selection,
            selector_qos,
        )
        self.create_timer(1.0 / update_rate_hz, self._update)

        self.get_logger().info(
            "Corner speed limiter ready | "
            f"lookahead={self.lookahead_distance_m:.2f} m "
            f"decel={self.braking_decel_m_s2:.2f} m/s^2 "
            f"default={self.lateral_accel_m_s2:.2f} m/s^2/"
            f"{self.min_corner_speed_m_s:.2f} m/s "
            f"heading={self.heading_hold_lateral_accel_m_s2:.2f} m/s^2/"
            f"{self.heading_hold_min_corner_speed_m_s:.2f} m/s "
            f"blind={'on' if self.blind_zone_enabled else 'off'}/"
            f"{self.blind_zone_speed_m_s:.2f} m/s "
            f"width={math.degrees(self.blind_zone_effective_half_width_rad):.1f} deg"
        )

    @staticmethod
    def _points(message: Path) -> list[Point2D]:
        points = []
        for pose in message.poses:
            if (
                not math.isfinite(pose.pose.position.x)
                or not math.isfinite(pose.pose.position.y)
                or pose.header.frame_id not in ("", message.header.frame_id)
            ):
                return []
            points.append(
                (float(pose.pose.position.x), float(pose.pose.position.y))
            )
        return points

    def _execution_path(self, message: Path) -> None:
        points = (
            self._points(message)
            if message.header.frame_id == self.path_frame_id
            else []
        )
        self.execution_path = points or None
        self.execution_path_received_s = time.monotonic()
        stamp = message.header.stamp
        self.execution_path_version = f"{stamp.sec}.{stamp.nanosec:09d}"
        # Apply the selected route's limit without waiting for the next timer.
        self._update()

    def _pose(self, message: PoseStamped) -> None:
        self.pose_x_m = float(message.pose.position.x)
        self.pose_y_m = float(message.pose.position.y)
        orientation = message.pose.orientation
        self.pose_yaw_rad = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        self.pose_received_s = time.monotonic()

    def _controller_selection(self, message: String) -> None:
        self.active_controller_id = str(message.data)

    def _corner_profile(self) -> tuple[str, float, float]:
        if self.active_controller_id == self.heading_hold_controller_id:
            return (
                "heading_hold",
                self.heading_hold_min_corner_speed_m_s,
                self.heading_hold_lateral_accel_m_s2,
            )
        return (
            "default",
            self.min_corner_speed_m_s,
            self.lateral_accel_m_s2,
        )

    def _decision(self, path_points: list[Point2D]) -> CornerSpeedDecision:
        _, min_corner_speed_m_s, lateral_accel_m_s2 = self._corner_profile()
        return corner_speed_limit(
            path_points,
            self.pose_x_m,
            self.pose_y_m,
            max_speed_m_s=self.max_speed_m_s,
            min_corner_speed_m_s=min_corner_speed_m_s,
            lateral_accel_m_s2=lateral_accel_m_s2,
            braking_decel_m_s2=self.braking_decel_m_s2,
            lookahead_distance_m=self.lookahead_distance_m,
            tangent_span_m=self.tangent_span_m,
            sample_step_m=self.sample_step_m,
            min_turn_angle_rad=self.min_turn_angle_rad,
            braking_margin_m=self.braking_margin_m,
        )

    def _blind_decision(self, path_points: list[Point2D]) -> BlindZoneSpeedDecision:
        if not self.blind_zone_enabled:
            return BlindZoneSpeedDecision(self.max_speed_m_s, 0.0, None, None)
        return blind_zone_speed_limit(
            path_points,
            self.pose_x_m,
            self.pose_y_m,
            self.pose_yaw_rad,
            max_speed_m_s=self.max_speed_m_s,
            blind_speed_m_s=self.blind_zone_speed_m_s,
            blind_angles_rad=self.blind_zone_angles_rad,
            half_width_rad=self.blind_zone_effective_half_width_rad,
            min_overlap_m=self.blind_zone_min_overlap_m,
            lookahead_distance_m=self.blind_zone_lookahead_m,
        )

    def _path_decisions(
        self, now_s: float
    ) -> list[tuple[str, CornerSpeedDecision, BlindZoneSpeedDecision]]:
        if (
            self.execution_path is None
            or now_s - self.execution_path_received_s > self.path_timeout_s
        ):
            return []
        return [
            (
                "execution_path",
                self._decision(self.execution_path),
                self._blind_decision(self.execution_path),
            )
        ]

    def _publish_limit(self, speed_limit_m_s: float) -> None:
        message = SpeedLimit()
        message.header.stamp = self.get_clock().now().to_msg()
        message.percentage = False
        if speed_limit_m_s >= self.max_speed_m_s - 1.0e-3:
            message.speed_limit = 0.0
        else:
            message.speed_limit = speed_limit_m_s
        self.limit_pub.publish(message)

    def _publish_status(
        self,
        now_s: float,
        source: str,
        decision: CornerSpeedDecision,
        blind_decision: BlindZoneSpeedDecision,
    ) -> None:
        if now_s - self.last_status_s < 0.20:
            return
        self.last_status_s = now_s
        message = String()
        profile, min_corner_speed_m_s, lateral_accel_m_s2 = self._corner_profile()
        message.data = json.dumps(
            {
                "source": source,
                "path_topic": self.execution_path_topic,
                "path_version": self.execution_path_version,
                "controller_id": self.active_controller_id,
                "profile": profile,
                "min_corner_speed_m_s": round(min_corner_speed_m_s, 3),
                "lateral_accel_m_s2": round(lateral_accel_m_s2, 3),
                "limited": min(
                    decision.speed_limit_m_s, blind_decision.speed_limit_m_s
                ) < self.max_speed_m_s - 1.0e-3,
                "limit_m_s": round(
                    min(
                        decision.speed_limit_m_s,
                        blind_decision.speed_limit_m_s,
                    ),
                    3,
                ),
                "corner_limit_m_s": round(decision.speed_limit_m_s, 3),
                "corner_distance_m": (
                    None
                    if decision.corner_distance_m is None
                    else round(decision.corner_distance_m, 3)
                ),
                "turn_deg": round(math.degrees(decision.turn_angle_rad), 1),
                "curvature_m_inv": round(decision.curvature_m_inv, 3),
                "blind_zone_limited": (
                    blind_decision.speed_limit_m_s < self.max_speed_m_s - 1.0e-3
                ),
                "blind_zone_limit_m_s": round(
                    blind_decision.speed_limit_m_s, 3
                ),
                "blind_zone_overlap_m": round(
                    blind_decision.overlap_distance_m, 3
                ),
                "blind_zone_angle_deg": (
                    None
                    if blind_decision.nearest_blind_angle_rad is None
                    else round(
                        math.degrees(blind_decision.nearest_blind_angle_rad), 1
                    )
                ),
                "path_body_heading_deg": (
                    None
                    if blind_decision.path_heading_rad is None
                    else round(math.degrees(blind_decision.path_heading_rad), 1)
                ),
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(message)

    def _update(self) -> None:
        now_s = time.monotonic()
        pose_fresh = (
            self.pose_x_m is not None
            and self.pose_y_m is not None
            and self.pose_yaw_rad is not None
            and all(
                math.isfinite(value)
                for value in (self.pose_x_m, self.pose_y_m, self.pose_yaw_rad)
            )
            and now_s - self.pose_received_s <= self.pose_timeout_s
        )
        if not pose_fresh:
            decision = CornerSpeedDecision(self.max_speed_m_s, None, 0.0, 0.0)
            blind_decision = self._unverified_blind_decision()
            self._publish_limit(blind_decision.speed_limit_m_s)
            self._publish_status(
                now_s, "no_fresh_pose", decision, blind_decision
            )
            return

        decisions = self._path_decisions(now_s)
        if not decisions:
            decision = CornerSpeedDecision(self.max_speed_m_s, None, 0.0, 0.0)
            blind_decision = self._unverified_blind_decision()
            self._publish_limit(blind_decision.speed_limit_m_s)
            self._publish_status(
                now_s, "no_fresh_path", decision, blind_decision
            )
            return

        source, decision, blind_decision = min(
            decisions,
            key=lambda item: min(
                item[1].speed_limit_m_s, item[2].speed_limit_m_s
            ),
        )
        combined_limit = min(
            decision.speed_limit_m_s, blind_decision.speed_limit_m_s
        )
        self._publish_limit(combined_limit)
        self._publish_status(now_s, source, decision, blind_decision)

    def _unverified_blind_decision(self) -> BlindZoneSpeedDecision:
        # SpeedLimit=0 releases the cap; it is not a safe fallback or a stop.
        limit = (
            min(self.max_speed_m_s, self.blind_zone_speed_m_s)
            if self.blind_zone_enabled
            else self.max_speed_m_s
        )
        return BlindZoneSpeedDecision(limit, 0.0, None, None)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CornerSpeedLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
