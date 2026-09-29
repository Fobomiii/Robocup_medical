#!/usr/bin/env python3
"""Apply task-specific approach envelopes before the final safety monitor."""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import UInt8

from .field_goals import load_field_goals
from .home_approach_core import limit_home_velocity


class HomeApproachLimiter(Node):
    """Apply soft braking envelopes while navigating home or to either bed."""

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
        self.field_config = str(self.declare_parameter("field_config", "").value)
        self.home_task_state = int(self.declare_parameter("home_task_state", 9).value)
        self.bed1_task_state = int(self.declare_parameter("bed1_task_state", 3).value)
        self.bed3_task_state = int(self.declare_parameter("bed3_task_state", 6).value)
        self.max_speed_m_s = float(
            self.declare_parameter("max_speed_m_s", 2.0).value
        )
        self.soft_decel_m_s2 = float(
            self.declare_parameter("soft_decel_m_s2", 1.425).value
        )
        self.terminal_speed_m_s = float(
            self.declare_parameter("terminal_speed_m_s", 0.10).value
        )
        self.terminal_distance_m = float(
            self.declare_parameter("terminal_distance_m", 0.10).value
        )
        self.bed_max_speed_m_s = float(
            self.declare_parameter("bed_max_speed_m_s", 2.0).value
        )
        self.bed_soft_decel_m_s2 = float(
            self.declare_parameter("bed_soft_decel_m_s2", 1.0).value
        )
        self.bed_terminal_speed_m_s = float(
            self.declare_parameter("bed_terminal_speed_m_s", 0.15).value
        )
        self.bed_terminal_distance_m = float(
            self.declare_parameter("bed_terminal_distance_m", 0.20).value
        )
        self.pose_timeout_s = float(
            self.declare_parameter("pose_timeout_s", 0.5).value
        )

        if not self.field_config:
            self.field_config = get_package_share_directory("obstacle_detector") + "/config/field_map.yaml"
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
            ),
            self.bed1_task_state: (
                *ros_goal("bed1"),
                self.bed_max_speed_m_s,
                self.bed_soft_decel_m_s2,
                self.bed_terminal_speed_m_s,
                self.bed_terminal_distance_m,
            ),
            self.bed3_task_state: (
                *ros_goal("bed3"),
                self.bed_max_speed_m_s,
                self.bed_soft_decel_m_s2,
                self.bed_terminal_speed_m_s,
                self.bed_terminal_distance_m,
            ),
        }
        if len(self.approach_profiles) != 3:
            raise ValueError("home and bed approach task states must be unique")

        self.task_state = -1
        self.pose_x_m: Optional[float] = None
        self.pose_y_m: Optional[float] = None
        self.last_pose_s = 0.0
        self.latest_cmd: Optional[Twist] = None
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self._cmd_callback, 10)
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_callback, 10)
        self.create_subscription(UInt8, self.task_state_topic, self._task_callback, 10)

        self.get_logger().info(
            f"Approach limiter ready | home={self.home_task_state}:"
            f"{self.soft_decel_m_s2:.2f} m/s^2 beds="
            f"{self.bed1_task_state},{self.bed3_task_state}:"
            f"{self.bed_soft_decel_m_s2:.2f} m/s^2"
        )

    @staticmethod
    def _zero_twist() -> Twist:
        return Twist()

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(self._zero_twist())

    def _task_callback(self, message: UInt8) -> None:
        previous = self.task_state
        self.task_state = int(message.data)
        previous_limited = previous in self.approach_profiles
        current_limited = self.task_state in self.approach_profiles
        if previous_limited and not current_limited:
            # Do not carry a pre-transition command into docking or task handoff.
            self._publish_zero()
        elif current_limited and self.latest_cmd is not None:
            self._publish_limited(self.latest_cmd)

    def _pose_callback(self, message: PoseStamped) -> None:
        self.pose_x_m = float(message.pose.position.x)
        self.pose_y_m = float(message.pose.position.y)
        self.last_pose_s = time.monotonic()
        if self.task_state in self.approach_profiles and self.latest_cmd is not None:
            self._publish_limited(self.latest_cmd)

    def _cmd_callback(self, message: Twist) -> None:
        self.latest_cmd = message
        if self.task_state not in self.approach_profiles:
            self.cmd_pub.publish(message)
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
            (
                target_x_m,
                target_y_m,
                max_speed_m_s,
                soft_decel_m_s2,
                terminal_speed_m_s,
                terminal_distance_m,
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
            )
        output = Twist()
        output.linear.x = limited_x
        output.linear.y = limited_y
        output.linear.z = message.linear.z
        output.angular.x = message.angular.x
        output.angular.y = message.angular.y
        output.angular.z = message.angular.z
        self.cmd_pub.publish(output)


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
