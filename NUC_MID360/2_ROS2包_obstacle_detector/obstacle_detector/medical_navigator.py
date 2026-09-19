#!/usr/bin/env python3
"""Field goal dispatcher for the Nav2 stack.

Translates the STM32's MSG_GOAL_REQUEST into a Nav2 NavigateToPose action and
publishes its status. stm32_bridge is the only process allowed to own the
serial port and forwards that status to the chassis.

Goals are written in the STM32 field frame (+X right, +Y forward, clockwise
yaw) because that is how the field is surveyed. The Nav2 side works in the ROS
map frame, so ``yaw_sign`` is applied here exactly as it is in the bridge.
"""

import json
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from .field_goals import load_field_goals
from .nav_protocol import (
    GOAL_NONE,
    NAV_ERROR,
    NAV_FOLLOWING,
    NAV_IDLE,
    NAV_REACHED,
    NAV_WAIT_PATH,
)


class MedicalNavigator(Node):
    def __init__(self) -> None:
        super().__init__("medical_navigator")

        default_map = os.path.join(
            get_package_share_directory("obstacle_detector"), "config", "field_map.yaml"
        )
        self.declare_parameter("map_config", default_map)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("yaw_sign", -1.0)
        self.declare_parameter("retry_limit", 1)
        self.declare_parameter("startup_rejection_retry_limit", 20)

        get = lambda name: self.get_parameter(name).value
        self.map_frame = str(get("map_frame"))
        self.yaw_sign = float(get("yaw_sign"))
        self.retry_limit = max(0, int(get("retry_limit")))
        self.startup_rejection_retry_limit = max(
            0, int(get("startup_rejection_retry_limit"))
        )

        self.goals_by_name, self.goals = load_field_goals(str(get("map_config")))

        self.nav_status = NAV_IDLE
        self.active_goal_id = GOAL_NONE
        self.active_request_id = 0
        self.attempt = 0
        self.rejection_retry_count = 0
        self.last_result = "idle"
        self.last_request_s = 0.0
        self.busy = False
        self.pending_goal = None
        self.lock = threading.Lock()

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, "/medical_nav/navigator_status", 10)
        self.plan_pub = self.create_publisher(Path, "/medical_nav/plan", latched)
        self.create_subscription(String, "/medical_nav/goal_request", self._goal_request, 10)
        self.create_subscription(Path, "/plan", self._nav2_plan, 10)

        self.create_timer(0.5, self._pending_goal_tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f"Navigator ready | goals={sorted(self.goals_by_name)} "
            f"| nav2={'up' if self.nav_client.server_is_ready() else 'starting'}"
        )

    # ------------------------------------------------------------------

    def _goal_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            goal_id = int(payload["goal_id"])
            request_id = int(payload.get("request_id", 0))
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(f"Bad goal request: {exc}")
            return
        self.dispatch(goal_id, request_id)

    def dispatch(self, goal_id: int, request_id: int = 0) -> bool:
        goal = self.goals.get(goal_id)
        if goal is None:
            self.get_logger().warn(f"Unknown goal id {goal_id}")
            with self.lock:
                self.active_goal_id = goal_id
                self.active_request_id = request_id
                self.nav_status = NAV_ERROR
                self.last_result = "unknown_goal"
                self.busy = False
                self.pending_goal = None
            self._publish_status()
            return False
        continuing_same_goal = False
        with self.lock:
            if self.busy and goal_id == self.active_goal_id:
                # The STM32 may reboot and issue a new request id while Nav2 is
                # already driving to the same destination. Keep the active goal,
                # but acknowledge the current request so status frames still match.
                self.active_request_id = request_id
                self.last_request_s = time.monotonic()
                continuing_same_goal = True
            else:
                self.active_goal_id = goal_id
                self.active_request_id = request_id
                self.attempt = 0
                self.rejection_retry_count = 0
                self.busy = True
                self.pending_goal = goal
                self.last_request_s = time.monotonic()
                self.nav_status = NAV_WAIT_PATH
        if continuing_same_goal:
            self._publish_status()
            return True
        self._publish_plan(Path())
        self._publish_status()
        self._send_goal(goal)
        return True

    def _send_goal(self, goal) -> None:
        if not self.nav_client.server_is_ready():
            with self.lock:
                self.pending_goal = goal
                self.nav_status = NAV_WAIT_PATH
                self.last_result = "waiting_for_nav2"
            self.get_logger().warn(
                "NavigateToPose server not ready; keeping goal queued",
                throttle_duration_sec=2.0,
            )
            return

        with self.lock:
            self.pending_goal = None

        target = PoseStamped()
        target.header.frame_id = self.map_frame
        target.header.stamp = self.get_clock().now().to_msg()
        # Field frame -> ROS map frame: x forward = field +Y, y left = field -X.
        target.pose.position.x = float(goal.y_mm) / 1000.0
        target.pose.position.y = -float(goal.x_mm) / 1000.0
        yaw = self.yaw_sign * math.radians(float(goal.yaw_deg))
        target.pose.orientation.z = math.sin(yaw * 0.5)
        target.pose.orientation.w = math.cos(yaw * 0.5)

        request = NavigateToPose.Goal()
        request.pose = target
        self.get_logger().info(
            f"Dispatching {goal.name} -> map({target.pose.position.x:.2f}, "
            f"{target.pose.position.y:.2f}) request={self.active_request_id}"
        )
        future = self.nav_client.send_goal_async(
            request, feedback_callback=self._feedback
        )
        future.add_done_callback(self._goal_accepted)

    def _pending_goal_tick(self) -> None:
        with self.lock:
            goal = self.pending_goal if self.busy else None
        if goal is not None and self.nav_client.server_is_ready():
            self._send_goal(goal)

    def _feedback(self, feedback) -> None:
        # Nav2 reports the remaining distance; keep the chassis lamp honest.
        if self.nav_status != NAV_FOLLOWING:
            self.nav_status = NAV_FOLLOWING

    def _goal_accepted(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            # During boot the action server is discoverable slightly before
            # bt_navigator reaches the active lifecycle state.  A request in
            # that window is rejected even though Nav2 becomes usable moments
            # later.  Keep the STM32 request alive and retry from the existing
            # pending-goal timer instead of putting the whole task into its
            # terminal navigation-error state.
            retry = False
            retry_count = 0
            with self.lock:
                goal = self.goals.get(self.active_goal_id)
                if (
                    self.busy
                    and goal is not None
                    and self.rejection_retry_count
                    < self.startup_rejection_retry_limit
                ):
                    self.rejection_retry_count += 1
                    retry_count = self.rejection_retry_count
                    self.pending_goal = goal
                    self.nav_status = NAV_WAIT_PATH
                    self.last_result = "waiting_for_nav2_activation"
                    retry = True
            if retry:
                self.get_logger().warn(
                    "Goal rejected while Nav2 is activating; "
                    f"queued retry {retry_count}/"
                    f"{self.startup_rejection_retry_limit}"
                )
                self._publish_status()
                return
            self.get_logger().warn("Goal rejected by Nav2 after startup retries")
            self._finish(NAV_ERROR, "rejected")
            return
        with self.lock:
            self.nav_status = NAV_FOLLOWING
            self.last_result = "following"
        self._publish_status()
        handle.get_result_async().add_done_callback(self._goal_finished)

    def _goal_finished(self, future) -> None:
        status = future.result()
        code = int(getattr(status, "status", 0))
        # action_msgs/GoalStatus: 4 = succeeded, 5 = canceled, 6 = aborted.
        if code == 4:
            self._finish(NAV_REACHED, "succeeded")
            return
        if code == 5:
            self._finish(NAV_ERROR, "canceled")
            return
        if self.attempt < self.retry_limit:
            self.attempt += 1
            with self.lock:
                self.nav_status = NAV_WAIT_PATH
            self.get_logger().warn(f"Nav2 failed (status {code}); retrying once")
            self._publish_status()
            self._send_goal(self.goals[self.active_goal_id])
            return
        self._finish(NAV_ERROR, f"failed_status_{code}")

    def _finish(self, nav_status: int, result: str) -> None:
        with self.lock:
            self.busy = False
            self.pending_goal = None
            self.nav_status = nav_status
            self.last_result = result
        self.get_logger().info(f"Navigation {result} (state={nav_status})")
        self._publish_status()

    def _nav2_plan(self, msg: Path) -> None:
        self.plan_pub.publish(msg)

    def _publish_plan(self, path: Path) -> None:
        self.plan_pub.publish(path)

    # ------------------------------------------------------------------

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "state": self.nav_status,
                "result": self.last_result,
                "goal_id": self.active_goal_id,
                "request_id": self.active_request_id,
                "busy": self.busy,
                "attempt": self.attempt,
                "rejection_retries": self.rejection_retry_count,
                "nav2_ready": self.nav_client.server_is_ready(),
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(message)

def main(args=None):
    rclpy.init(args=args)
    node = MedicalNavigator()
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
