#!/usr/bin/env python3
"""Fail-safe final velocity relay with point-cloud-verified omni escape."""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String

from .nav_protocol import (
    NAV_FOLLOWING,
    TASK_NAV_BED1,
    TASK_NAV_BED3,
    TASK_NAV_HOME,
    TASK_NAV_NURSE,
)
from .omni_escape_core import (
    BlockedIntentLatch,
    EscapeDirectionDecision,
    StallDecision,
    StallDetector,
    choose_escape_direction,
    direction_is_swept_clear,
    path_length,
)
from .qos_profiles import low_latency_sensor_qos


NAVIGATION_TASK_STATES = frozenset(
    (TASK_NAV_NURSE, TASK_NAV_BED1, TASK_NAV_BED3, TASK_NAV_HOME)
)


class OmniEscape(Node):
    """Relay normal commands and override only a continuously verified stall."""

    PASS_THROUGH = "pass_through"
    ESCAPING = "escaping"
    SETTLING = "settling"
    COOLDOWN = "cooldown"

    def __init__(self) -> None:
        super().__init__("omni_escape")
        declare = self.declare_parameter
        self.enabled = bool(declare("enabled", True).value)
        self.input_topic = str(
            declare("input_topic", "/cmd_vel_collision_safe").value
        )
        self.upstream_topic = str(
            declare("upstream_topic", "/cmd_vel_home_limited").value
        )
        self.output_topic = str(declare("output_topic", "/cmd_vel_safe").value)
        self.cloud_topic = str(
            declare("cloud_topic", "/livox/lidar_safety").value
        )
        self.odom_topic = str(declare("odom_topic", "/odom").value)
        self.navigator_status_topic = str(
            declare(
                "navigator_status_topic", "/medical_nav/navigator_status"
            ).value
        )
        self.path_topic = str(
            declare("path_topic", "/transformed_global_plan").value
        )
        self.escape_command_topic = str(
            declare(
                "escape_command_topic", "/medical_nav/omni_escape_cmd_vel"
            ).value
        )
        self.status_topic = str(
            declare("status_topic", "/medical_nav/omni_escape_status").value
        )

        self.update_rate_hz = float(declare("update_rate_hz", 20.0).value)
        self.stall_duration_s = float(declare("stall_duration_s", 4.0).value)
        self.blocked_intent_hold_s = float(
            declare("blocked_intent_hold_s", 6.0).value
        )
        self.minimum_upstream_speed_m_s = float(
            declare("minimum_upstream_speed_m_s", 0.18).value
        )
        self.stationary_speed_m_s = float(
            declare("stationary_speed_m_s", 0.05).value
        )
        self.safe_command_stop_m_s = float(
            declare("safe_command_stop_m_s", 0.03).value
        )
        self.maximum_stall_displacement_m = float(
            declare("maximum_stall_displacement_m", 0.08).value
        )
        self.minimum_remaining_path_m = float(
            declare("minimum_remaining_path_m", 0.50).value
        )
        self.footprint_radius_m = float(
            declare("footprint_radius_m", 0.265).value
        )
        self.travel_distance_m = float(
            declare("travel_distance_m", 0.30).value
        )
        self.clearance_margin_m = float(
            declare("clearance_margin_m", 0.03).value
        )
        self.minimum_clearance_gain_m = float(
            declare("minimum_clearance_gain_m", 0.04).value
        )
        self.sample_step_m = float(declare("sample_step_m", 0.05).value)
        self.direction_samples = int(declare("direction_samples", 16).value)
        self.evidence_min_points = int(
            declare("evidence_min_points", 8).value
        )
        self.max_collision_points = int(
            declare("max_collision_points", 3).value
        )
        self.nearby_radius_m = float(declare("nearby_radius_m", 0.75).value)
        self.escape_speed_m_s = float(
            declare("escape_speed_m_s", 0.20).value
        )
        self.escape_accel_m_s2 = float(
            declare("escape_accel_m_s2", 0.80).value
        )
        self.escape_decel_m_s2 = float(
            declare("escape_decel_m_s2", 1.20).value
        )
        self.escape_timeout_s = float(
            declare("escape_timeout_s", 2.50).value
        )
        self.settle_duration_s = float(
            declare("settle_duration_s", 0.25).value
        )
        self.cooldown_duration_s = float(
            declare("cooldown_duration_s", 1.0).value
        )
        self.maximum_attempts = int(declare("maximum_attempts", 2).value)
        self.command_timeout_s = float(
            declare("command_timeout_s", 0.30).value
        )
        self.cloud_timeout_s = float(declare("cloud_timeout_s", 0.20).value)
        self.odom_timeout_s = float(declare("odom_timeout_s", 0.20).value)
        self.status_timeout_s = float(declare("status_timeout_s", 1.20).value)
        self.path_timeout_s = float(declare("path_timeout_s", 1.50).value)

        if self.update_rate_hz <= 0.0:
            raise ValueError("update rate must be positive")
        if self.escape_speed_m_s <= 0.0:
            raise ValueError("escape speed must be positive")
        if self.escape_accel_m_s2 <= 0.0 or self.escape_decel_m_s2 <= 0.0:
            raise ValueError("escape acceleration limits must be positive")
        if self.maximum_attempts <= 0:
            raise ValueError("maximum attempts must be positive")
        if self.evidence_min_points <= self.max_collision_points:
            raise ValueError(
                "evidence point count must exceed the collision point allowance"
            )

        self.stall_detector = StallDetector(
            duration_s=self.stall_duration_s,
            minimum_upstream_speed_m_s=self.minimum_upstream_speed_m_s,
            maximum_safe_speed_m_s=self.safe_command_stop_m_s,
            maximum_measured_speed_m_s=self.stationary_speed_m_s,
            maximum_displacement_m=self.maximum_stall_displacement_m,
        )
        self.blocked_intent = BlockedIntentLatch(
            hold_s=self.blocked_intent_hold_s,
            minimum_upstream_speed_m_s=self.minimum_upstream_speed_m_s,
            maximum_safe_speed_m_s=self.safe_command_stop_m_s,
        )
        self.last_stall_decision = StallDecision(
            False, False, 0.0, 0.0, "startup"
        )
        self.state = self.PASS_THROUGH
        self.reason = "startup"
        self.state_started_s = time.monotonic()
        self.attempt_count = 0
        self.last_input: Optional[Twist] = None
        self.last_input_s: Optional[float] = None
        self.last_upstream: Optional[Twist] = None
        self.last_upstream_s: Optional[float] = None
        self.last_odom: Optional[Odometry] = None
        self.last_odom_s: Optional[float] = None
        self.last_points: Optional[np.ndarray] = None
        self.last_cloud_s: Optional[float] = None
        self.last_cloud_frame = ""
        self.last_status_s: Optional[float] = None
        self.navigator_state = -1
        self.navigator_busy = False
        self.task_state = -1
        self.last_path_s: Optional[float] = None
        self.remaining_path_m = 0.0
        self.escape_start_xy = (0.0, 0.0)
        self.escape_direction = (0.0, 0.0)
        self.escape_velocity_m_s = 0.0
        self.last_update_s = time.monotonic()
        self.last_decision: Optional[EscapeDirectionDecision] = None
        self.last_status_publish_s = 0.0

        normal_qos = QoSProfile(depth=10)
        path_qos = QoSProfile(depth=1)
        self.create_subscription(
            Twist, self.input_topic, self._input_callback, normal_qos
        )
        self.create_subscription(
            Twist, self.upstream_topic, self._upstream_callback, normal_qos
        )
        self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self._cloud_callback,
            low_latency_sensor_qos(),
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            low_latency_sensor_qos(),
        )
        self.create_subscription(
            String,
            self.navigator_status_topic,
            self._navigator_status_callback,
            normal_qos,
        )
        self.create_subscription(Path, self.path_topic, self._path_callback, path_qos)
        self.output_publisher = self.create_publisher(
            Twist, self.output_topic, normal_qos
        )
        self.escape_publisher = self.create_publisher(
            Twist, self.escape_command_topic, normal_qos
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, normal_qos
        )
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self._update)
        self.get_logger().info(
            "OmniEscape ready | "
            f"stall={self.stall_duration_s:.1f}s "
            f"distance={self.travel_distance_m:.2f}m "
            f"speed={self.escape_speed_m_s:.2f}m/s"
        )

    @staticmethod
    def _speed(command: Optional[Twist]) -> float:
        if command is None:
            return 0.0
        return math.hypot(float(command.linear.x), float(command.linear.y))

    @staticmethod
    def _copy_twist(command: Twist) -> Twist:
        output = Twist()
        output.linear.x = command.linear.x
        output.linear.y = command.linear.y
        output.linear.z = command.linear.z
        output.angular.x = command.angular.x
        output.angular.y = command.angular.y
        output.angular.z = command.angular.z
        return output

    def _measured_speed(self) -> float:
        if self.last_odom is None:
            return 0.0
        twist = self.last_odom.twist.twist
        return math.hypot(float(twist.linear.x), float(twist.linear.y))

    def _input_callback(self, message: Twist) -> None:
        self.last_input = self._copy_twist(message)
        now_s = time.monotonic()
        self.last_input_s = now_s
        self._update_blocked_intent(now_s)
        if self.state in (self.PASS_THROUGH, self.COOLDOWN):
            self.output_publisher.publish(self.last_input)

    def _upstream_callback(self, message: Twist) -> None:
        self.last_upstream = self._copy_twist(message)
        now_s = time.monotonic()
        self.last_upstream_s = now_s
        self._update_blocked_intent(now_s)

    def _update_blocked_intent(self, now_s: float) -> None:
        if self.last_input is None or self.last_upstream is None:
            return
        if not self._fresh(self.last_input_s, self.command_timeout_s, now_s):
            return
        if not self._fresh(self.last_upstream_s, self.command_timeout_s, now_s):
            return
        self.blocked_intent.update(
            now_s=now_s,
            upstream_x_m_s=float(self.last_upstream.linear.x),
            upstream_y_m_s=float(self.last_upstream.linear.y),
            safe_speed_m_s=self._speed(self.last_input),
        )

    def _effective_upstream_vector(self, now_s: float) -> tuple[float, float]:
        if (
            self._fresh(self.last_upstream_s, self.command_timeout_s, now_s)
            and self._speed(self.last_upstream) >= self.minimum_upstream_speed_m_s
        ):
            assert self.last_upstream is not None
            return (
                float(self.last_upstream.linear.x),
                float(self.last_upstream.linear.y),
            )
        latched = self.blocked_intent.get(now_s)
        return (0.0, 0.0) if latched is None else latched

    def _cloud_callback(self, message: PointCloud2) -> None:
        self.last_cloud_s = time.monotonic()
        self.last_cloud_frame = message.header.frame_id.lstrip("/")
        points = pc2.read_points(
            message, field_names=("x", "y"), skip_nans=True
        )
        if points.size == 0:
            self.last_points = np.empty((0, 2), dtype=np.float64)
            return
        self.last_points = np.column_stack((points["x"], points["y"])).astype(
            np.float64, copy=False
        )

    def _odom_callback(self, message: Odometry) -> None:
        self.last_odom = message
        self.last_odom_s = time.monotonic()

    def _navigator_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.navigator_state = int(payload.get("state", -1))
            self.navigator_busy = bool(payload.get("busy", False))
            self.task_state = int(payload.get("task_state", -1))
            self.last_status_s = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("Ignoring malformed navigator status")

    def _path_callback(self, message: Path) -> None:
        points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        ]
        self.remaining_path_m = path_length(points)
        self.last_path_s = time.monotonic()

    @staticmethod
    def _fresh(stamp_s: Optional[float], timeout_s: float, now_s: float) -> bool:
        return stamp_s is not None and 0.0 <= now_s - stamp_s <= timeout_s

    def _static_guard_reason(self, now_s: float) -> str:
        blocked_intent_active = self.blocked_intent.get(now_s) is not None
        if not self.enabled:
            return "disabled"
        if (
            not self._fresh(self.last_input_s, self.command_timeout_s, now_s)
            and not blocked_intent_active
        ):
            return "collision_command_stale"
        if (
            not self._fresh(self.last_upstream_s, self.command_timeout_s, now_s)
            and not blocked_intent_active
        ):
            return "upstream_command_stale"
        if not self._fresh(self.last_odom_s, self.odom_timeout_s, now_s):
            return "odom_stale"
        if not self._fresh(self.last_cloud_s, self.cloud_timeout_s, now_s):
            return "cloud_stale"
        if self.last_cloud_frame != "base_link":
            return "cloud_not_in_base_link"
        if not self._fresh(self.last_status_s, self.status_timeout_s, now_s):
            return "navigator_status_stale"
        if not self._fresh(self.last_path_s, self.path_timeout_s, now_s):
            return "path_stale"
        if self.navigator_state != NAV_FOLLOWING or not self.navigator_busy:
            return "not_following"
        if self.task_state not in NAVIGATION_TASK_STATES:
            return "not_navigation_task"
        if self.remaining_path_m < self.minimum_remaining_path_m:
            return "near_goal"
        if self.attempt_count >= self.maximum_attempts:
            return "attempt_limit"
        if self.last_points is None or len(self.last_points) < self.evidence_min_points:
            return "insufficient_points"
        ranges = np.hypot(self.last_points[:, 0], self.last_points[:, 1])
        if (
            int(np.count_nonzero(ranges <= self.nearby_radius_m))
            < self.evidence_min_points
        ):
            return "no_nearby_obstacle"
        return "stall_inputs_ready"

    def _stall_decision(self, now_s: float, guard_reason: str) -> StallDecision:
        if self.last_odom is None:
            self.stall_detector.reset()
            return StallDecision(False, False, 0.0, 0.0, "odom_missing")
        position = self.last_odom.pose.pose.position
        effective_vx, effective_vy = self._effective_upstream_vector(now_s)
        return self.stall_detector.update(
            now_s=now_s,
            eligible=guard_reason == "stall_inputs_ready",
            upstream_speed_m_s=math.hypot(effective_vx, effective_vy),
            safe_speed_m_s=self._speed(self.last_input),
            measured_speed_m_s=self._measured_speed(),
            position_x_m=float(position.x),
            position_y_m=float(position.y),
        )

    def _start_escape(self, now_s: float) -> bool:
        assert self.last_points is not None
        upstream_vx, upstream_vy = self._effective_upstream_vector(now_s)
        decision = choose_escape_direction(
            self.last_points,
            float(upstream_vx),
            float(upstream_vy),
            footprint_radius_m=self.footprint_radius_m,
            travel_distance_m=self.travel_distance_m,
            clearance_margin_m=self.clearance_margin_m,
            minimum_clearance_gain_m=self.minimum_clearance_gain_m,
            sample_step_m=self.sample_step_m,
            direction_samples=self.direction_samples,
            evidence_min_points=self.evidence_min_points,
            max_collision_points=self.max_collision_points,
            nearby_radius_m=self.nearby_radius_m,
        )
        self.last_decision = decision
        self.stall_detector.reset()
        if not decision.valid:
            self.reason = decision.reason
            return False
        assert self.last_odom is not None
        position = self.last_odom.pose.pose.position
        self.escape_start_xy = (float(position.x), float(position.y))
        self.escape_direction = (decision.direction_x, decision.direction_y)
        self.escape_velocity_m_s = 0.0
        self.attempt_count += 1
        self.state = self.ESCAPING
        self.state_started_s = now_s
        self.last_update_s = now_s
        self.reason = "escape_started"
        self.get_logger().warning(
            "OmniEscape attempt "
            f"{self.attempt_count}/{self.maximum_attempts}: "
            f"dir=({decision.direction_x:.3f},{decision.direction_y:.3f}) "
            f"clearance={decision.current_clearance_m:.3f}->"
            f"{decision.final_clearance_m:.3f} m"
        )
        return True

    def _escape_guard_reason(self, now_s: float) -> str:
        checks = (
            (self.enabled, "disabled"),
            (
                self._fresh(self.last_odom_s, self.odom_timeout_s, now_s),
                "odom_stale",
            ),
            (
                self._fresh(self.last_cloud_s, self.cloud_timeout_s, now_s),
                "cloud_stale",
            ),
            (self.last_cloud_frame == "base_link", "cloud_not_in_base_link"),
            (
                self._fresh(self.last_status_s, self.status_timeout_s, now_s),
                "navigator_status_stale",
            ),
            (
                self._fresh(self.last_path_s, self.path_timeout_s, now_s),
                "path_stale",
            ),
            (
                self.navigator_state == NAV_FOLLOWING and self.navigator_busy,
                "not_following",
            ),
            (self.task_state in NAVIGATION_TASK_STATES, "not_navigation_task"),
            (self.remaining_path_m >= self.minimum_remaining_path_m, "near_goal"),
            (
                self.last_points is not None
                and len(self.last_points) >= self.evidence_min_points,
                "insufficient_points",
            ),
        )
        for valid, reason in checks:
            if not valid:
                return reason
        return "escape_allowed"

    def _distance_from_escape_start(self) -> float:
        if self.last_odom is None:
            return 0.0
        position = self.last_odom.pose.pose.position
        return math.hypot(
            float(position.x) - self.escape_start_xy[0],
            float(position.y) - self.escape_start_xy[1],
        )

    def _enter_settling(self, now_s: float, reason: str) -> None:
        self.state = self.SETTLING
        self.state_started_s = now_s
        self.escape_velocity_m_s = 0.0
        self.reason = reason
        self._publish_escape_command(0.0)

    def _publish_escape_command(self, speed_m_s: float) -> None:
        command = Twist()
        command.linear.x = self.escape_direction[0] * speed_m_s
        command.linear.y = self.escape_direction[1] * speed_m_s
        command.angular.z = 0.0
        self.output_publisher.publish(command)
        self.escape_publisher.publish(command)

    def _update_escape(self, now_s: float) -> None:
        guard = self._escape_guard_reason(now_s)
        if guard != "escape_allowed":
            self._enter_settling(now_s, f"escape_aborted:{guard}")
            return
        elapsed_s = now_s - self.state_started_s
        if elapsed_s >= self.escape_timeout_s:
            self._enter_settling(now_s, "escape_timeout")
            return
        travelled_m = self._distance_from_escape_start()
        remaining_m = max(0.0, self.travel_distance_m - travelled_m)
        if remaining_m <= 0.01:
            self._enter_settling(now_s, "escape_distance_reached")
            return
        assert self.last_points is not None
        if not direction_is_swept_clear(
            self.last_points,
            self.escape_direction[0],
            self.escape_direction[1],
            footprint_radius_m=self.footprint_radius_m,
            travel_distance_m=remaining_m,
            clearance_margin_m=self.clearance_margin_m,
            max_collision_points=self.max_collision_points,
        ):
            self._enter_settling(now_s, "escape_path_blocked")
            return

        dt_s = min(0.20, max(0.0, now_s - self.last_update_s))
        self.last_update_s = now_s
        braking_speed = math.sqrt(2.0 * self.escape_decel_m_s2 * remaining_m)
        desired_speed = min(self.escape_speed_m_s, braking_speed)
        if desired_speed >= self.escape_velocity_m_s:
            self.escape_velocity_m_s = min(
                desired_speed,
                self.escape_velocity_m_s + self.escape_accel_m_s2 * dt_s,
            )
        else:
            self.escape_velocity_m_s = max(
                desired_speed,
                self.escape_velocity_m_s - self.escape_decel_m_s2 * dt_s,
            )
        self.reason = "escaping"
        self._publish_escape_command(self.escape_velocity_m_s)

    def _publish_status(self, now_s: float, force: bool = False) -> None:
        if not force and now_s - self.last_status_publish_s < 0.10:
            return
        self.last_status_publish_s = now_s
        decision = self.last_decision
        status = {
            "ready": True,
            "enabled": self.enabled,
            "state": self.state,
            "reason": self.reason,
            "attempt": self.attempt_count,
            "maximum_attempts": self.maximum_attempts,
            "stall_s": round(self.last_stall_decision.elapsed_s, 3),
            "stall_displacement_m": round(
                self.last_stall_decision.displacement_m, 3
            ),
            "remaining_path_m": round(self.remaining_path_m, 3),
            "input_speed_m_s": round(self._speed(self.last_input), 3),
            "upstream_speed_m_s": round(self._speed(self.last_upstream), 3),
            "effective_intent_speed_m_s": round(
                math.hypot(*self._effective_upstream_vector(now_s)), 3
            ),
            "blocked_intent_age_s": None
            if self.blocked_intent.age_s(now_s) is None
            else round(self.blocked_intent.age_s(now_s), 3),
            "measured_speed_m_s": round(self._measured_speed(), 3),
            "escape_speed_m_s": round(self.escape_velocity_m_s, 3),
            "escape_distance_m": round(self._distance_from_escape_start(), 3)
            if self.state == self.ESCAPING
            else 0.0,
            "direction": [
                round(self.escape_direction[0], 4),
                round(self.escape_direction[1], 4),
            ],
            "decision": None
            if decision is None
            else {
                "valid": decision.valid,
                "reason": decision.reason,
                "current_clearance_m": decision.current_clearance_m,
                "final_clearance_m": decision.final_clearance_m,
                "minimum_clearance_m": decision.minimum_clearance_m,
                "nearby_point_count": decision.nearby_point_count,
                "rejected_direction_count": decision.rejected_direction_count,
            },
        }
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self.status_publisher.publish(message)

    def _update(self) -> None:
        now_s = time.monotonic()
        if self.state == self.ESCAPING:
            self._update_escape(now_s)
        elif self.state == self.SETTLING:
            self._publish_escape_command(0.0)
            if now_s - self.state_started_s >= self.settle_duration_s:
                self.state = self.COOLDOWN
                self.state_started_s = now_s
                self.reason = "replanning_cooldown"
        else:
            if not self._fresh(self.last_input_s, self.command_timeout_s, now_s):
                self.output_publisher.publish(Twist())
            guard_reason = self._static_guard_reason(now_s)
            self.last_stall_decision = self._stall_decision(now_s, guard_reason)
            self.reason = (
                guard_reason
                if guard_reason != "stall_inputs_ready"
                else self.last_stall_decision.reason
            )
            if (
                self._speed(self.last_input) > 0.10
                and self._measured_speed() > 0.10
            ):
                self.attempt_count = 0
                self.last_decision = None
            if self.state == self.COOLDOWN:
                self.stall_detector.reset()
                if now_s - self.state_started_s >= self.cooldown_duration_s:
                    self.state = self.PASS_THROUGH
                    self.reason = "pass_through"
            elif self.last_stall_decision.ready:
                self._start_escape(now_s)
        self._publish_status(now_s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OmniEscape()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.output_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
