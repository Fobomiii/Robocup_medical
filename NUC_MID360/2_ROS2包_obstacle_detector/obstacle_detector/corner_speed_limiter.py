#!/usr/bin/env python3
"""Publish curvature-aware speed limits and a shadow reference profile."""

from __future__ import annotations

import json
import math
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

from .corner_speed_core import (
    CornerLimitReleaseLatch,
    CornerSpeedDecision,
    Point2D,
    corner_speed_limit,
    reference_speed_profile,
)


class CornerSpeedLimiter(Node):
    """Coordinate braking limits while DWB remains a single fixed profile."""

    def __init__(self) -> None:
        super().__init__("corner_speed_limiter")

        self.enabled = bool(self.declare_parameter("enabled", False).value)
        self.raw_path_topic = str(
            self.declare_parameter("raw_path_topic", "/plan").value
        )
        self.smoothed_path_topic = str(
            self.declare_parameter("smoothed_path_topic", "/plan_smoothed").value
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
        self.reference_profile_topic = str(
            self.declare_parameter(
                "reference_profile_topic",
                "/medical_nav/reference_speed_profile",
            ).value
        )
        self.reference_status_topic = str(
            self.declare_parameter(
                "reference_status_topic",
                "/medical_nav/reference_speed_status",
            ).value
        )
        self.odom_topic = str(
            self.declare_parameter("profile_odom_topic", "/odom").value
        )
        self.max_speed_m_s = float(
            self.declare_parameter("max_speed_m_s", 3.0).value
        )
        self.min_corner_speed_m_s = float(
            self.declare_parameter("min_corner_speed_m_s", 0.50).value
        )
        self.max_corner_speed_m_s = float(
            self.declare_parameter("max_corner_speed_m_s", 1.00).value
        )
        self.lateral_accel_m_s2 = float(
            self.declare_parameter("lateral_accel_m_s2", 2.30).value
        )
        self.braking_decel_m_s2 = float(
            self.declare_parameter("braking_decel_m_s2", 2.30).value
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
        self.reaction_time_s = float(
            self.declare_parameter("reaction_time_s", 0.20).value
        )
        self.release_hold_s = float(
            self.declare_parameter("release_hold_s", 0.50).value
        )
        self.straight_confirmation_s = float(
            self.declare_parameter("straight_confirmation_s", 0.20).value
        )
        self.path_timeout_s = float(
            self.declare_parameter("path_timeout_s", 1.50).value
        )
        self.pose_timeout_s = float(
            self.declare_parameter("pose_timeout_s", 0.50).value
        )
        self.smoothing_wait_s = float(
            self.declare_parameter("smoothing_wait_s", 0.25).value
        )
        update_rate_hz = float(
            self.declare_parameter("update_rate_hz", 20.0).value
        )
        self.profile_shadow_enabled = bool(
            self.declare_parameter("profile_shadow_enabled", True).value
        )
        self.profile_max_speed_m_s = float(
            self.declare_parameter("profile_max_speed_m_s", 3.0).value
        )
        self.profile_min_curve_speed_m_s = float(
            self.declare_parameter("profile_min_curve_speed_m_s", 0.50).value
        )
        self.profile_lateral_accel_m_s2 = float(
            self.declare_parameter("profile_lateral_accel_m_s2", 2.30).value
        )
        self.profile_accel_m_s2 = float(
            self.declare_parameter("profile_accel_m_s2", 4.0).value
        )
        self.profile_braking_decel_m_s2 = float(
            self.declare_parameter("profile_braking_decel_m_s2", 2.0).value
        )
        self.profile_reaction_time_s = float(
            self.declare_parameter("profile_reaction_time_s", 0.20).value
        )
        self.profile_braking_margin_m = float(
            self.declare_parameter("profile_braking_margin_m", 0.20).value
        )
        self.profile_lookahead_distance_m = float(
            self.declare_parameter("profile_lookahead_distance_m", 6.0).value
        )
        self.profile_sample_step_m = float(
            self.declare_parameter("profile_sample_step_m", 0.10).value
        )
        self.profile_curvature_span_m = float(
            self.declare_parameter("profile_curvature_span_m", 0.50).value
        )
        self.profile_curvature_filter_window_m = float(
            self.declare_parameter(
                "profile_curvature_filter_window_m", 0.20
            ).value
        )
        self.profile_command_lookahead_m = float(
            self.declare_parameter("profile_command_lookahead_m", 0.25).value
        )
        self.profile_odom_timeout_s = float(
            self.declare_parameter("profile_odom_timeout_s", 0.50).value
        )
        profile_publish_rate_hz = float(
            self.declare_parameter("profile_publish_rate_hz", 5.0).value
        )

        self.raw_path: Optional[list[Point2D]] = None
        self.smoothed_path: Optional[list[Point2D]] = None
        self.raw_path_received_s = 0.0
        self.smoothed_path_received_s = 0.0
        self.pose_x_m: Optional[float] = None
        self.pose_y_m: Optional[float] = None
        self.pose_received_s = 0.0
        self.odom_speed_m_s: Optional[float] = None
        self.odom_received_s = 0.0
        self.last_status_s = 0.0
        self.release_latch = CornerLimitReleaseLatch(
            release_hold_s=self.release_hold_s,
            straight_confirmation_s=self.straight_confirmation_s,
        )
        self.last_corner_decision: Optional[CornerSpeedDecision] = None

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.limit_pub = self.create_publisher(SpeedLimit, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.reference_profile_pub = self.create_publisher(
            Float32MultiArray, self.reference_profile_topic, 10
        )
        self.reference_status_pub = self.create_publisher(
            String, self.reference_status_topic, 10
        )
        self.create_subscription(Path, self.raw_path_topic, self._raw_path, path_qos)
        self.create_subscription(
            Path, self.smoothed_path_topic, self._smoothed_path, path_qos
        )
        self.create_subscription(PoseStamped, self.pose_topic, self._pose, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom, 10)
        self.create_timer(1.0 / update_rate_hz, self._update)
        self.create_timer(
            1.0 / profile_publish_rate_hz,
            self._publish_reference_profile,
        )
        self.get_logger().info(
            "Corner speed limiter ready | "
            f"enabled={self.enabled} "
            f"lookahead={self.lookahead_distance_m:.2f} m "
            f"decel={self.braking_decel_m_s2:.2f} m/s^2 "
            f"reaction={self.reaction_time_s:.2f} s "
            f"release={self.release_hold_s:.2f} s/"
            f"straight={self.straight_confirmation_s:.2f} s "
            f"default={self.lateral_accel_m_s2:.2f} m/s^2/"
            f"{self.min_corner_speed_m_s:.2f}-"
            f"{self.max_corner_speed_m_s:.2f} m/s"
        )
        self.get_logger().info(
            "Reference speed shadow ready | "
            f"enabled={self.profile_shadow_enabled} "
            f"rate={profile_publish_rate_hz:.1f} Hz "
            f"lookahead={self.profile_lookahead_distance_m:.1f} m "
            "control_output=none"
        )

    @staticmethod
    def _points(message: Path) -> list[Point2D]:
        return [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
            if math.isfinite(pose.pose.position.x)
            and math.isfinite(pose.pose.position.y)
        ]

    def _raw_path(self, message: Path) -> None:
        self.raw_path = self._points(message)
        self.raw_path_received_s = time.monotonic()

    def _smoothed_path(self, message: Path) -> None:
        self.smoothed_path = self._points(message)
        self.smoothed_path_received_s = time.monotonic()

    def _pose(self, message: PoseStamped) -> None:
        self.pose_x_m = float(message.pose.position.x)
        self.pose_y_m = float(message.pose.position.y)
        self.pose_received_s = time.monotonic()

    def _odom(self, message: Odometry) -> None:
        vx_m_s = float(message.twist.twist.linear.x)
        vy_m_s = float(message.twist.twist.linear.y)
        if not math.isfinite(vx_m_s) or not math.isfinite(vy_m_s):
            return
        self.odom_speed_m_s = math.hypot(vx_m_s, vy_m_s)
        self.odom_received_s = time.monotonic()

    def _reference_path(
        self, now_s: float
    ) -> tuple[str, Optional[list[Point2D]]]:
        raw_fresh = (
            self.raw_path is not None
            and now_s - self.raw_path_received_s <= self.path_timeout_s
        )
        smoothed_fresh = (
            self.smoothed_path is not None
            and now_s - self.smoothed_path_received_s <= self.path_timeout_s
        )
        if smoothed_fresh and (
            not raw_fresh
            or self.smoothed_path_received_s >= self.raw_path_received_s
        ):
            return "smoothed", self.smoothed_path
        if raw_fresh:
            return "raw_shadow_fallback", self.raw_path
        if smoothed_fresh:
            return "smoothed", self.smoothed_path
        return "no_fresh_path", None

    def _reference_parameters(self) -> dict[str, float]:
        return {
            "max_speed_m_s": self.profile_max_speed_m_s,
            "min_curve_speed_m_s": self.profile_min_curve_speed_m_s,
            "lateral_accel_m_s2": self.profile_lateral_accel_m_s2,
            "accel_m_s2": self.profile_accel_m_s2,
            "braking_decel_m_s2": self.profile_braking_decel_m_s2,
            "reaction_time_s": self.profile_reaction_time_s,
            "braking_margin_m": self.profile_braking_margin_m,
            "lookahead_distance_m": self.profile_lookahead_distance_m,
            "sample_step_m": self.profile_sample_step_m,
            "curvature_span_m": self.profile_curvature_span_m,
            "curvature_filter_window_m": (
                self.profile_curvature_filter_window_m
            ),
            "command_lookahead_m": self.profile_command_lookahead_m,
        }

    def _publish_reference_status(self, **fields) -> None:
        message = String()
        message.data = json.dumps(
            {
                "shadow_mode": True,
                "affects_control": False,
                "enabled": self.profile_shadow_enabled,
                **fields,
            },
            separators=(",", ":"),
        )
        self.reference_status_pub.publish(message)

    def _publish_empty_reference_profile(self, reason: str, source: str) -> None:
        self.reference_profile_pub.publish(Float32MultiArray())
        self._publish_reference_status(
            valid=False,
            reason=reason,
            source=source,
            sample_count=0,
            fields=[
                "s",
                "x",
                "y",
                "curvature",
                "radius",
                "curve_speed",
                "braking_speed",
                "reference_speed",
            ],
            parameters=self._reference_parameters(),
        )

    def _publish_reference_profile(self) -> None:
        now_s = time.monotonic()
        if not self.profile_shadow_enabled:
            self._publish_empty_reference_profile("disabled", "disabled")
            return
        pose_fresh = (
            self.pose_x_m is not None
            and self.pose_y_m is not None
            and now_s - self.pose_received_s <= self.pose_timeout_s
        )
        if not pose_fresh:
            self._publish_empty_reference_profile("no_fresh_pose", "none")
            return
        odom_fresh = (
            self.odom_speed_m_s is not None
            and now_s - self.odom_received_s <= self.profile_odom_timeout_s
        )
        if not odom_fresh:
            self._publish_empty_reference_profile("no_fresh_odom", "none")
            return
        source, path_points = self._reference_path(now_s)
        if path_points is None:
            self._publish_empty_reference_profile("no_fresh_path", source)
            return

        profile = reference_speed_profile(
            path_points,
            self.pose_x_m,
            self.pose_y_m,
            self.odom_speed_m_s,
            **self._reference_parameters(),
        )
        if not profile.samples:
            self._publish_empty_reference_profile("path_too_short", source)
            return

        fields = (
            "s",
            "x",
            "y",
            "curvature",
            "radius",
            "curve_speed",
            "braking_speed",
            "reference_speed",
        )
        message = Float32MultiArray()
        message.layout.dim = [
            MultiArrayDimension(
                label="samples",
                size=len(profile.samples),
                stride=len(profile.samples) * len(fields),
            ),
            MultiArrayDimension(
                label="fields:" + ",".join(fields),
                size=len(fields),
                stride=len(fields),
            ),
        ]
        message.data = [
            value
            for sample in profile.samples
            for value in (
                sample.distance_m,
                sample.x_m,
                sample.y_m,
                sample.curvature_m_inv,
                sample.radius_m,
                sample.curve_speed_m_s,
                sample.braking_speed_m_s,
                sample.reference_speed_m_s,
            )
        ]
        self.reference_profile_pub.publish(message)

        most_restrictive = min(
            profile.samples,
            key=lambda sample: sample.reference_speed_m_s,
        )
        maximum_curvature = max(
            sample.curvature_m_inv for sample in profile.samples
        )
        self._publish_reference_status(
            valid=True,
            reason="ok",
            source=source,
            sample_count=len(profile.samples),
            measured_speed_m_s=round(self.odom_speed_m_s, 4),
            command_speed_m_s=round(profile.command_speed_m_s, 4),
            command_distance_m=round(profile.command_distance_m, 4),
            minimum_reference_speed_m_s=round(
                most_restrictive.reference_speed_m_s, 4
            ),
            minimum_reference_distance_m=round(
                most_restrictive.distance_m, 4
            ),
            maximum_curvature_m_inv=round(maximum_curvature, 5),
            fields=list(fields),
            parameters=self._reference_parameters(),
        )

    def _corner_profile(self) -> tuple[str, float, float, float]:
        return (
            "default",
            self.min_corner_speed_m_s,
            self.max_corner_speed_m_s,
            self.lateral_accel_m_s2,
        )

    def _decision(self, path_points: list[Point2D]) -> CornerSpeedDecision:
        (
            _,
            min_corner_speed_m_s,
            max_corner_speed_m_s,
            lateral_accel_m_s2,
        ) = self._corner_profile()
        return corner_speed_limit(
            path_points,
            self.pose_x_m,
            self.pose_y_m,
            max_speed_m_s=self.max_speed_m_s,
            min_corner_speed_m_s=min_corner_speed_m_s,
            max_corner_speed_m_s=max_corner_speed_m_s,
            lateral_accel_m_s2=lateral_accel_m_s2,
            braking_decel_m_s2=self.braking_decel_m_s2,
            lookahead_distance_m=self.lookahead_distance_m,
            tangent_span_m=self.tangent_span_m,
            sample_step_m=self.sample_step_m,
            min_turn_angle_rad=self.min_turn_angle_rad,
            braking_margin_m=self.braking_margin_m,
            reaction_time_s=self.reaction_time_s,
        )

    def _path_decisions(
        self, now_s: float
    ) -> list[tuple[str, CornerSpeedDecision]]:
        def decisions_for(source: str, path: list[Point2D]):
            return (source, self._decision(path))

        raw_fresh = (
            self.raw_path is not None
            and now_s - self.raw_path_received_s <= self.path_timeout_s
        )
        smoothed_fresh = (
            self.smoothed_path is not None
            and now_s - self.smoothed_path_received_s <= self.path_timeout_s
        )
        if raw_fresh and self.smoothed_path_received_s >= self.raw_path_received_s:
            return [decisions_for("smoothed", self.smoothed_path)]
        if raw_fresh and now_s - self.raw_path_received_s < self.smoothing_wait_s:
            decisions = [decisions_for("raw", self.raw_path)]
            if smoothed_fresh:
                decisions.append(
                    decisions_for("previous_smoothed", self.smoothed_path)
                )
            return decisions
        if raw_fresh:
            return [decisions_for("raw_fallback", self.raw_path)]
        if smoothed_fresh:
            return [decisions_for("smoothed", self.smoothed_path)]
        return []

    def _publish_limit(self, speed_limit_m_s: float) -> None:
        message = SpeedLimit()
        message.header.stamp = self.get_clock().now().to_msg()
        message.percentage = False
        if speed_limit_m_s >= self.max_speed_m_s - 1.0e-3:
            # Nav2 interprets zero as removing the cap, not commanding a stop.
            message.speed_limit = 0.0
        else:
            message.speed_limit = speed_limit_m_s
        self.limit_pub.publish(message)

    def _apply_release_hold(
        self,
        now_s: float,
        source: str,
        decision: CornerSpeedDecision,
        *,
        straight_path_confirmed: bool,
    ) -> tuple[str, CornerSpeedDecision, bool]:
        corner_detected = decision.corner_distance_m is not None
        if corner_detected:
            self.last_corner_decision = decision
        hold_active = self.release_latch.update(
            now_s,
            corner_detected=corner_detected,
            straight_path_confirmed=straight_path_confirmed,
        )
        if not hold_active:
            if not self.release_latch.armed:
                self.last_corner_decision = None
            return source, decision, False

        previous = self.last_corner_decision
        held_speed_m_s = self.max_corner_speed_m_s
        if previous is not None and previous.corner_speed_m_s is not None:
            held_speed_m_s = min(held_speed_m_s, previous.corner_speed_m_s)
        held_decision = CornerSpeedDecision(
            held_speed_m_s,
            None,
            0.0 if previous is None else previous.turn_angle_rad,
            0.0 if previous is None else previous.curvature_m_inv,
            held_speed_m_s,
        )
        return "corner_release_hold", held_decision, True

    def _publish_status(
        self,
        now_s: float,
        source: str,
        decision: CornerSpeedDecision,
        path_data_valid: bool,
        release_hold_active: bool = False,
    ) -> None:
        if now_s - self.last_status_s < 0.20:
            return
        self.last_status_s = now_s
        message = String()
        (
            profile,
            min_corner_speed_m_s,
            max_corner_speed_m_s,
            lateral_accel_m_s2,
        ) = self._corner_profile()
        message.data = json.dumps(
            {
                "enabled": self.enabled,
                "source": source,
                "profile": profile,
                "min_corner_speed_m_s": round(min_corner_speed_m_s, 3),
                "max_corner_speed_m_s": round(max_corner_speed_m_s, 3),
                "lateral_accel_m_s2": round(lateral_accel_m_s2, 3),
                "reaction_time_s": round(self.reaction_time_s, 3),
                "release_hold_s": round(self.release_hold_s, 3),
                "straight_confirmation_s": round(
                    self.straight_confirmation_s, 3
                ),
                "release_hold_active": release_hold_active,
                "path_data_valid": path_data_valid,
                "reaction_distance_m": round(
                    decision.speed_limit_m_s * self.reaction_time_s,
                    3,
                ),
                "limited": (
                    decision.speed_limit_m_s < self.max_speed_m_s - 1.0e-3
                ),
                "limit_m_s": round(decision.speed_limit_m_s, 3),
                "corner_limit_m_s": (
                    None
                    if decision.corner_speed_m_s is None
                    else round(decision.corner_speed_m_s, 3)
                ),
                "corner_distance_m": (
                    None
                    if decision.corner_distance_m is None
                    else round(decision.corner_distance_m, 3)
                ),
                "turn_deg": round(math.degrees(decision.turn_angle_rad), 1),
                "curvature_m_inv": round(decision.curvature_m_inv, 3),
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(message)

    def _update(self) -> None:
        now_s = time.monotonic()
        if not self.enabled:
            # Repeat the reset so a restarted controller cannot retain an old
            # speed cap.
            decision = CornerSpeedDecision(self.max_speed_m_s, None, 0.0, 0.0)
            self.release_latch.reset()
            self.last_corner_decision = None
            self._publish_limit(decision.speed_limit_m_s)
            self._publish_status(
                now_s,
                "disabled",
                decision,
                False,
            )
            return

        pose_fresh = (
            self.pose_x_m is not None
            and self.pose_y_m is not None
            and now_s - self.pose_received_s <= self.pose_timeout_s
        )
        if not pose_fresh:
            decision = CornerSpeedDecision(self.max_speed_m_s, None, 0.0, 0.0)
            source = "no_fresh_pose"
            straight_path_confirmed = False
            path_data_valid = False
        else:
            decisions = self._path_decisions(now_s)
            if decisions:
                source, decision = min(
                    decisions,
                    key=lambda item: item[1].speed_limit_m_s,
                )
                straight_path_confirmed = decision.corner_distance_m is None
                path_data_valid = True
            else:
                decision = CornerSpeedDecision(
                    self.max_speed_m_s, None, 0.0, 0.0
                )
                source = "no_fresh_path"
                straight_path_confirmed = False
                path_data_valid = False

        source, decision, release_hold_active = self._apply_release_hold(
            now_s,
            source,
            decision,
            straight_path_confirmed=straight_path_confirmed,
        )
        self._publish_limit(decision.speed_limit_m_s)
        self._publish_status(
            now_s,
            source,
            decision,
            path_data_valid,
            release_hold_active,
        )


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
