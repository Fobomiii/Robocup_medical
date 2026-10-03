#!/usr/bin/env python3
"""Apply task-specific approach envelopes before the final safety monitor."""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String, UInt8

from .field_goals import load_field_goals
from .home_approach_core import (
    limit_bed_body_velocity,
    limit_home_velocity,
    scale_planar_velocity,
)


class HomeApproachLimiter(Node):
    """Apply the global planar cap plus home/bed braking envelopes."""

    def __init__(self) -> None:
        super().__init__("home_approach_limiter")

        self.input_topic = str(
            self.declare_parameter("input_topic", "/cmd_vel").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/cmd_vel_home_limited").value
        )
        self.pose_topic = str(
            self.declare_parameter("pose_topic", "/medical_nav/robot_pose").value
        )
        self.task_state_topic = str(
            self.declare_parameter("task_state_topic", "/medical_nav/task_state").value
        )
        self.wheel_odom_topic = str(
            self.declare_parameter(
                "wheel_odom_topic", "/medical_nav/wheel_odom"
            ).value
        )
        self.status_topic = str(
            self.declare_parameter(
                "status_topic", "/medical_nav/bed_approach_status"
            ).value
        )
        self.field_config = str(self.declare_parameter("field_config", "").value)
        self.home_task_state = int(self.declare_parameter("home_task_state", 9).value)
        self.bed1_task_state = int(self.declare_parameter("bed1_task_state", 3).value)
        self.bed3_task_state = int(self.declare_parameter("bed3_task_state", 6).value)
        self.max_speed_m_s = float(
            self.declare_parameter("max_speed_m_s", 3.0).value
        )
        self.soft_decel_m_s2 = float(
            self.declare_parameter("soft_decel_m_s2", 4.9875).value
        )
        self.terminal_speed_m_s = float(
            self.declare_parameter("terminal_speed_m_s", 0.10).value
        )
        self.terminal_distance_m = float(
            self.declare_parameter("terminal_distance_m", 0.10).value
        )
        self.decel_start_distance_m = float(
            self.declare_parameter("decel_start_distance_m", 1.00).value
        )
        self.bed_max_speed_m_s = float(
            self.declare_parameter("bed_max_speed_m_s", 3.0).value
        )
        self.bed_forward_decel_m_s2 = float(
            self.declare_parameter("bed_forward_decel_m_s2", 2.0).value
        )
        self.bed_side_decel_m_s2 = float(
            self.declare_parameter("bed_side_decel_m_s2", 2.2).value
        )
        self.bed_reaction_time_s = float(
            self.declare_parameter("bed_reaction_time_s", 0.22).value
        )
        self.bed_braking_margin_m = float(
            self.declare_parameter("bed_braking_margin_m", 0.03).value
        )
        self.pose_timeout_s = float(
            self.declare_parameter("pose_timeout_s", 0.5).value
        )
        self.wheel_odom_timeout_s = float(
            self.declare_parameter("wheel_odom_timeout_s", 0.15).value
        )

        if not self.field_config:
            self.field_config = (
                get_package_share_directory("obstacle_detector")
                + "/config/field_map.yaml"
            )
        goals_by_name, _ = load_field_goals(self.field_config)
        required_goals = ("home", "bed1", "bed3")
        missing_goals = [name for name in required_goals if name not in goals_by_name]
        if missing_goals:
            raise ValueError(
                f"field map is missing approach goals {missing_goals}: {self.field_config}"
            )

        def ros_goal(name: str) -> tuple[float, float]:
            goal = goals_by_name[name]
            return goal.y_mm / 1000.0, -goal.x_mm / 1000.0

        self.approach_profiles = {
            self.home_task_state: (
                *ros_goal("home"),
                self.max_speed_m_s,
                self.soft_decel_m_s2,
                self.terminal_speed_m_s,
                self.terminal_distance_m,
                self.decel_start_distance_m,
            ),
        }
        self.bed_profiles = {
            self.bed1_task_state: (*ros_goal("bed1"), 1.0),
            self.bed3_task_state: (*ros_goal("bed3"), -1.0),
        }
        if len({self.home_task_state, *self.bed_profiles}) != 3:
            raise ValueError("home and bed approach task states must be unique")

        self.task_state = -1
        self.pose_x_m: Optional[float] = None
        self.pose_y_m: Optional[float] = None
        self.pose_yaw_rad = 0.0
        self.last_pose_s = 0.0
        self.measured_world_vx_m_s: Optional[float] = None
        self.measured_world_vy_m_s: Optional[float] = None
        self.last_wheel_odom_s = 0.0
        self.latest_cmd: Optional[Twist] = None
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Twist, self.input_topic, self._cmd_callback, 10)
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_callback, 10)
        self.create_subscription(UInt8, self.task_state_topic, self._task_callback, 10)
        self.create_subscription(
            Odometry, self.wheel_odom_topic, self._wheel_odom_callback, 10
        )

        self.get_logger().info(
            f"Approach limiter ready | home={self.home_task_state}:"
            f"start={self.decel_start_distance_m:.2f} m "
            f"({self.soft_decel_m_s2:.2f} m/s^2 equivalent) beds="
            f"{self.bed1_task_state},{self.bed3_task_state}:axis-aware "
            f"forward={self.bed_forward_decel_m_s2:.2f} m/s^2 "
            f"side={self.bed_side_decel_m_s2:.2f} m/s^2 "
            f"delay={self.bed_reaction_time_s:.2f} s"
        )

    @staticmethod
    def _zero_twist() -> Twist:
        return Twist()

    @staticmethod
    def _with_planar_velocity(message: Twist, vx: float, vy: float) -> Twist:
        output = Twist()
        output.linear.x = vx
        output.linear.y = vy
        output.linear.z = message.linear.z
        output.angular.x = message.angular.x
        output.angular.y = message.angular.y
        output.angular.z = message.angular.z
        return output

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(self._zero_twist())

    def _task_callback(self, message: UInt8) -> None:
        previous = self.task_state
        self.task_state = int(message.data)
        previous_limited = (
            previous in self.approach_profiles
            or previous in self.bed_profiles
        )
        current_limited = (
            self.task_state in self.approach_profiles
            or self.task_state in self.bed_profiles
        )
        if previous_limited and not current_limited:
            # Do not carry a pre-transition command into docking or task handoff.
            self._publish_zero()
        elif current_limited and self.latest_cmd is not None:
            self._publish_limited(self.latest_cmd)

    def _pose_callback(self, message: PoseStamped) -> None:
        self.pose_x_m = float(message.pose.position.x)
        self.pose_y_m = float(message.pose.position.y)
        orientation = message.pose.orientation
        self.pose_yaw_rad = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.last_pose_s = time.monotonic()
        if (
            self.task_state in self.approach_profiles
            or self.task_state in self.bed_profiles
        ) and self.latest_cmd is not None:
            self._publish_limited(self.latest_cmd)

    def _wheel_odom_callback(self, message: Odometry) -> None:
        valid = (
            message.twist.covariance[0] < 1.0e5
            and message.twist.covariance[7] < 1.0e5
        )
        if not valid:
            self.measured_world_vx_m_s = None
            self.measured_world_vy_m_s = None
            self.last_wheel_odom_s = 0.0
            return
        forward = float(message.twist.twist.linear.x)
        left = float(message.twist.twist.linear.y)
        cosine = math.cos(self.pose_yaw_rad)
        sine = math.sin(self.pose_yaw_rad)
        self.measured_world_vx_m_s = cosine * forward - sine * left
        self.measured_world_vy_m_s = sine * forward + cosine * left
        self.last_wheel_odom_s = time.monotonic()

    def _cmd_callback(self, message: Twist) -> None:
        self.latest_cmd = message
        if (
            self.task_state not in self.approach_profiles
            and self.task_state not in self.bed_profiles
        ):
            limited_x, limited_y = scale_planar_velocity(
                message.linear.x, message.linear.y, self.max_speed_m_s
            )
            self.cmd_pub.publish(
                self._with_planar_velocity(message, limited_x, limited_y)
            )
            return
        self._publish_limited(message)

    def _publish_limited(self, message: Twist) -> None:
        pose_fresh = (
            self.pose_x_m is not None
            and self.pose_y_m is not None
            and time.monotonic() - self.last_pose_s <= self.pose_timeout_s
        )
        if not pose_fresh:
            limited_x, limited_y = 0.0, 0.0
        else:
            if self.task_state in self.bed_profiles:
                target_x_m, target_y_m, side_toward_sign = self.bed_profiles[
                    self.task_state
                ]
                wheel_fresh = (
                    self.measured_world_vx_m_s is not None
                    and self.measured_world_vy_m_s is not None
                    and time.monotonic() - self.last_wheel_odom_s
                    <= self.wheel_odom_timeout_s
                )
                measured_vx = self.measured_world_vx_m_s if wheel_fresh else None
                measured_vy = self.measured_world_vy_m_s if wheel_fresh else None
                limited_x, limited_y, decision = limit_bed_body_velocity(
                    message.linear.x,
                    message.linear.y,
                    measured_vx,
                    measured_vy,
                    self.pose_yaw_rad,
                    self.pose_x_m,
                    self.pose_y_m,
                    target_x_m=target_x_m,
                    target_y_m=target_y_m,
                    side_toward_sign=side_toward_sign,
                    max_speed_m_s=self.bed_max_speed_m_s,
                    forward_braking_decel_m_s2=self.bed_forward_decel_m_s2,
                    side_braking_decel_m_s2=self.bed_side_decel_m_s2,
                    reaction_time_s=self.bed_reaction_time_s,
                    safety_margin_m=self.bed_braking_margin_m,
                )
                status = String()
                status.data = json.dumps(
                    {
                        "task_state": self.task_state,
                        "wheel_fresh": wheel_fresh,
                        "pose_x_m": self.pose_x_m,
                        "pose_y_m": self.pose_y_m,
                        "pose_yaw_rad": self.pose_yaw_rad,
                        "target_x_m": target_x_m,
                        "target_y_m": target_y_m,
                        "side_toward_sign": side_toward_sign,
                        "input_body_vx_m_s": message.linear.x,
                        "input_body_vy_m_s": message.linear.y,
                        "output_body_vx_m_s": limited_x,
                        "output_body_vy_m_s": limited_y,
                        "measured_world_vx_m_s": measured_vx,
                        "measured_world_vy_m_s": measured_vy,
                        **decision,
                    },
                    separators=(",", ":"),
                )
                self.status_pub.publish(status)
            else:
                (
                    target_x_m,
                    target_y_m,
                    max_speed_m_s,
                    soft_decel_m_s2,
                    terminal_speed_m_s,
                    terminal_distance_m,
                    decel_start_distance_m,
                ) = self.approach_profiles[self.task_state]
                limited_x, limited_y, _ = limit_home_velocity(
                    message.linear.x,
                    message.linear.y,
                    self.pose_x_m,
                    self.pose_y_m,
                    home_x_m=target_x_m,
                    home_y_m=target_y_m,
                    max_speed_m_s=max_speed_m_s,
                    soft_decel_m_s2=soft_decel_m_s2,
                    terminal_speed_m_s=terminal_speed_m_s,
                    terminal_distance_m=terminal_distance_m,
                    decel_start_distance_m=decel_start_distance_m,
                )
        self.cmd_pub.publish(
            self._with_planar_velocity(message, limited_x, limited_y)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HomeApproachLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
