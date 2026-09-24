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

from .field_goals import FieldGoal, load_field_goals
from .nav_protocol import (
    GOAL_NONE,
    NAV_ERROR,
    NAV_FOLLOWING,
    NAV_IDLE,
    NAV_REACHED,
    NAV_WAIT_PATH,
    SCAN_CONTEXT_ORDER,
)
from .nurse_scan_core import (
    field_yaw_toward,
    load_nurse_scan_config,
    next_nurse_viewpoint_index,
)


class MedicalNavigator(Node):
    def __init__(self) -> None:
        super().__init__("medical_navigator")

        default_map = os.path.join(
            get_package_share_directory("obstacle_detector"), "config", "field_map.yaml"
        )
        default_nurse_bt = os.path.join(
            get_package_share_directory("obstacle_detector"),
            "config",
            "navigate_to_pose_nurse_fast.xml",
        )
        self.declare_parameter("map_config", default_map)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("yaw_sign", -1.0)
        self.declare_parameter("retry_limit", 1)
        self.declare_parameter("startup_rejection_retry_limit", 20)
        self.declare_parameter("goal_handoff_timeout_s", 1.0)
        self.declare_parameter("nurse_bt_xml", default_nurse_bt)

        get = lambda name: self.get_parameter(name).value
        self.map_frame = str(get("map_frame"))
        self.yaw_sign = float(get("yaw_sign"))
        self.retry_limit = max(0, int(get("retry_limit")))
        self.startup_rejection_retry_limit = max(
            0, int(get("startup_rejection_retry_limit"))
        )
        self.goal_handoff_timeout_s = max(
            0.2, float(get("goal_handoff_timeout_s"))
        )
        self.nurse_bt_xml = str(get("nurse_bt_xml"))
        if not os.path.isfile(self.nurse_bt_xml):
            raise FileNotFoundError(
                f"nurse-station behavior tree is missing: {self.nurse_bt_xml}"
            )

        map_config = str(get("map_config"))
        self.goals_by_name, self.goals = load_field_goals(map_config)
        self.nurse_scan_config = load_nurse_scan_config(map_config)
        self.nurse_goal_id = self.goals_by_name["nurse"].goal_id

        self.nav_status = NAV_IDLE
        self.active_goal_id = GOAL_NONE
        self.active_request_id = 0
        self.attempt = 0
        self.rejection_retry_count = 0
        self.last_result = "idle"
        self.last_request_s = 0.0
        self.busy = False
        self.pending_goal = None
        self.active_goal_handle = None
        self.cancel_goal_handle = None
        self.goal_generation = 0
        self.waiting_for_cancel = False
        self.cancel_started_s = 0.0
        self.cancel_attempt = 0
        self.robot_field_pose_mm = None
        self.nurse_scan_mode = "inactive"
        self.nurse_viewpoint_index = 0
        self.nurse_dwell_deadline_s = 0.0
        self.nurse_action_started_s = 0.0
        self.nurse_cancel_started_s = 0.0
        self.nurse_cancel_attempt = 0
        self.nurse_next_goal = None
        self.nurse_qr_seen = False
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
        self.create_subscription(String, "/medical_nav/scan_result", self._scan_result, 10)
        self.create_subscription(
            PoseStamped, "/medical_nav/robot_pose", self._robot_pose, 10
        )
        self.create_subscription(Path, "/plan", self._nav2_plan, 10)

        self.create_timer(0.5, self._pending_goal_tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f"Navigator ready | goals={sorted(self.goals_by_name)} "
            f"| nav2={'up' if self.nav_client.server_is_ready() else 'starting'}"
        )

    # ------------------------------------------------------------------

    def _reset_nurse_scan_locked(self, active: bool) -> None:
        self.nurse_scan_mode = "approach" if active else "inactive"
        self.nurse_viewpoint_index = 0
        self.nurse_dwell_deadline_s = 0.0
        self.nurse_action_started_s = 0.0
        self.nurse_cancel_started_s = 0.0
        self.nurse_cancel_attempt = 0
        self.nurse_next_goal = None
        self.nurse_qr_seen = False

    def _nurse_goal_at(self, name: str, x_mm: float, y_mm: float) -> FieldGoal:
        yaw_deg = field_yaw_toward(
            x_mm,
            y_mm,
            self.nurse_scan_config.qr_x_mm,
            self.nurse_scan_config.qr_y_mm,
        )
        return FieldGoal(
            goal_id=self.nurse_goal_id,
            name=name,
            label=name,
            x_mm=x_mm,
            y_mm=y_mm,
            yaw_deg=yaw_deg,
        )

    def _robot_pose(self, message: PoseStamped) -> None:
        field_x_mm = -float(message.pose.position.y) * 1000.0
        field_y_mm = float(message.pose.position.x) * 1000.0
        stop_handle = None
        generation = 0
        with self.lock:
            self.robot_field_pose_mm = (field_x_mm, field_y_mm)
            if not (
                self.busy
                and self.active_goal_id == self.nurse_goal_id
                and self.nurse_scan_mode == "approach"
                and self.nav_status == NAV_FOLLOWING
                and self.active_goal_handle is not None
                and self.nurse_scan_config.zone.contains(field_x_mm, field_y_mm)
            ):
                return
            self.nurse_next_goal = self._nurse_goal_at(
                "nurse_scan_entry", field_x_mm, field_y_mm
            )
            self.nurse_scan_mode = "stopping_for_entry"
            self.nav_status = NAV_WAIT_PATH
            self.last_result = "nurse_scan_zone_stopping"
            stop_handle = self.active_goal_handle
            generation = self.goal_generation
        self._publish_status()
        self._request_nurse_cancel(stop_handle, generation)

    def _scan_result(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            context = int(payload["context"])
        except (ValueError, KeyError, TypeError):
            return
        if context != SCAN_CONTEXT_ORDER:
            return

        stop_handle = None
        generation = 0
        with self.lock:
            if not (
                self.busy
                and self.active_goal_id == self.nurse_goal_id
                and self.nurse_scan_mode != "inactive"
            ):
                return
            self.nurse_qr_seen = True
            self.pending_goal = None
            self.nurse_next_goal = None
            self.nurse_dwell_deadline_s = 0.0
            self.nurse_action_started_s = 0.0
            self.nav_status = NAV_WAIT_PATH
            self.last_result = "nurse_qr_seen_waiting_for_stm32"
            stop_handle = self.active_goal_handle
            generation = self.goal_generation
            self.nurse_scan_mode = (
                "stopping_for_qr" if stop_handle is not None else "qr_hold"
            )
        self._publish_status()
        if stop_handle is not None:
            self._request_nurse_cancel(stop_handle, generation)

    def _request_nurse_cancel(self, handle, generation: int) -> None:
        if handle is None:
            return
        with self.lock:
            if (
                generation != self.goal_generation
                or handle is not self.active_goal_handle
                or self.nurse_scan_mode
                not in {
                    "stopping_for_entry",
                    "stopping_for_qr",
                    "skipping_viewpoint",
                }
            ):
                return
            self.nurse_cancel_started_s = time.monotonic()
            self.nurse_cancel_attempt += 1
            attempt = self.nurse_cancel_attempt
        try:
            future = handle.cancel_goal_async()
            future.add_done_callback(
                lambda completed: self._nurse_cancel_response(
                    completed, handle, generation, attempt
                )
            )
        except Exception as exc:
            self.get_logger().error(
                f"Cannot stop nurse scan motion (attempt {attempt}): {exc}"
            )

    def _nurse_cancel_response(
        self, future, handle, generation: int, attempt: int
    ) -> None:
        try:
            accepted = bool(getattr(future.result(), "goals_canceling", ()))
        except Exception as exc:
            self.get_logger().warn(
                f"Nurse scan cancellation response failed (attempt {attempt}): {exc}"
            )
            return
        with self.lock:
            current = (
                generation == self.goal_generation
                and handle is self.active_goal_handle
                and self.nurse_scan_mode
                in {
                    "stopping_for_entry",
                    "stopping_for_qr",
                    "skipping_viewpoint",
                }
            )
        if current and not accepted:
            self.get_logger().warn(
                "Nurse scan goal did not accept cancellation; motion remains blocked"
            )

    def _hold_nurse_for_qr(self, generation: int) -> None:
        with self.lock:
            if generation != self.goal_generation:
                return
            self.active_goal_handle = None
            self.pending_goal = None
            self.nurse_scan_mode = "qr_hold"
            self.nurse_cancel_started_s = 0.0
            self.nav_status = NAV_WAIT_PATH
            self.last_result = "nurse_qr_seen_waiting_for_stm32"
        self._publish_status()

    def _begin_nurse_dwell(self, generation: int, result: str) -> None:
        with self.lock:
            if generation != self.goal_generation:
                return
            if self.nurse_qr_seen:
                hold_for_qr = self.nurse_qr_seen
            else:
                hold_for_qr = False
                self.active_goal_handle = None
                self.pending_goal = None
                self.nurse_scan_mode = "dwell"
                self.nurse_dwell_deadline_s = (
                    time.monotonic() + self.nurse_scan_config.dwell_s
                )
                self.nurse_action_started_s = 0.0
                self.nurse_cancel_started_s = 0.0
                self.nav_status = NAV_WAIT_PATH
                self.last_result = result
        if hold_for_qr:
            self._hold_nurse_for_qr(generation)
        else:
            self._publish_status()

    def _queue_nurse_scan_goal(
        self, goal: FieldGoal, generation: int, result: str
    ) -> None:
        with self.lock:
            if generation != self.goal_generation:
                return
            if self.nurse_qr_seen:
                hold_for_qr = self.nurse_qr_seen
            else:
                hold_for_qr = False
                self.active_goal_handle = None
                self.pending_goal = goal
                self.nurse_scan_mode = "moving_to_viewpoint"
                self.nurse_action_started_s = 0.0
                self.nurse_cancel_started_s = 0.0
                self.nav_status = NAV_WAIT_PATH
                self.last_result = result
        if hold_for_qr:
            self._hold_nurse_for_qr(generation)
            return
        self._publish_status()
        self._send_goal(goal, generation)

    def _send_next_nurse_viewpoint(self, generation: int) -> None:
        with self.lock:
            if generation != self.goal_generation or self.nurse_qr_seen:
                hold_for_qr = self.nurse_qr_seen
                viewpoint = None
            else:
                hold_for_qr = False
                selected_index = next_nurse_viewpoint_index(
                    self.nurse_viewpoint_index,
                    len(self.nurse_scan_config.viewpoints),
                )
                viewpoint = self.nurse_scan_config.viewpoints[selected_index]
                self.nurse_viewpoint_index = selected_index + 1
        if hold_for_qr:
            self._hold_nurse_for_qr(generation)
            return
        goal = self._nurse_goal_at(viewpoint.name, viewpoint.x_mm, viewpoint.y_mm)
        self._queue_nurse_scan_goal(
            goal, generation, f"moving_to_{viewpoint.name}"
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
                self.cancel_goal_handle = None
                self.waiting_for_cancel = False
                self.cancel_started_s = 0.0
                self.cancel_attempt = 0
            self._publish_status()
            return False
        continuing_same_goal = False
        repeated_terminal_request = False
        previous_handle = None
        generation = 0
        with self.lock:
            if (
                not self.busy
                and goal_id == self.active_goal_id
                and request_id == self.active_request_id
                and self.nav_status in (NAV_REACHED, NAV_ERROR)
            ):
                repeated_terminal_request = True
            elif self.busy and goal_id == self.active_goal_id:
                # The STM32 may reboot and issue a new request id while Nav2 is
                # already driving to the same destination. Keep the active goal,
                # but acknowledge the current request so status frames still match.
                self.active_request_id = request_id
                self.last_request_s = time.monotonic()
                continuing_same_goal = True
            else:
                previous_handle = self.active_goal_handle or self.cancel_goal_handle
                self.active_goal_handle = None
                self.cancel_goal_handle = previous_handle
                self.goal_generation += 1
                generation = self.goal_generation
                self._reset_nurse_scan_locked(goal_id == self.nurse_goal_id)
                self.active_goal_id = goal_id
                self.active_request_id = request_id
                self.attempt = 0
                self.rejection_retry_count = 0
                self.busy = True
                self.pending_goal = goal
                self.last_request_s = time.monotonic()
                self.nav_status = NAV_WAIT_PATH
                self.waiting_for_cancel = previous_handle is not None
                self.cancel_started_s = (
                    time.monotonic() if previous_handle is not None else 0.0
                )
                self.cancel_attempt = 0
                self.last_result = (
                    "canceling_previous_goal"
                    if previous_handle is not None
                    else "dispatching"
                )
        if continuing_same_goal or repeated_terminal_request:
            self._publish_status()
            return True
        self._publish_plan(Path())
        self._publish_status()
        if previous_handle is not None:
            previous_handle.get_result_async().add_done_callback(
                lambda completed: self._previous_goal_terminal(completed, generation)
            )
            self._request_previous_goal_cancel(previous_handle, generation)
        elif goal_id == self.nurse_goal_id:
            self._send_next_nurse_viewpoint(generation)
        else:
            self._send_goal(goal, generation)
        return True

    def _request_previous_goal_cancel(self, previous_handle, generation: int) -> None:
        with self.lock:
            if (
                generation != self.goal_generation
                or not self.waiting_for_cancel
                or previous_handle is not self.cancel_goal_handle
            ):
                return
            self.cancel_started_s = time.monotonic()
            self.cancel_attempt += 1
            attempt = self.cancel_attempt
            self.last_result = f"canceling_previous_goal_{attempt}"
        try:
            cancel_future = previous_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda completed: self._previous_goal_cancel_response(
                    completed, previous_handle, generation, attempt
                )
            )
        except Exception as exc:
            self.get_logger().error(
                f"Cannot request previous-goal cancellation (attempt {attempt}): {exc}"
            )

    def _previous_goal_cancel_response(
        self, future, previous_handle, generation: int, attempt: int
    ) -> None:
        try:
            response = future.result()
            canceling = bool(getattr(response, "goals_canceling", ()))
        except Exception as exc:
            self.get_logger().warn(
                f"Previous-goal cancellation response failed "
                f"(attempt {attempt}): {exc}"
            )
            return
        with self.lock:
            current = (
                generation == self.goal_generation
                and self.waiting_for_cancel
                and previous_handle is self.cancel_goal_handle
            )
        if current and not canceling:
            self.get_logger().warn(
                "Previous goal did not accept cancellation; keeping chassis "
                "stopped until the action reaches a terminal state"
            )

    def _previous_goal_terminal(self, future, generation: int) -> None:
        try:
            status = future.result()
            code = int(getattr(status, "status", 0))
            result = f"previous_goal_terminal_{code}"
        except Exception as exc:
            self.get_logger().warn(f"Cannot read previous-goal result: {exc}")
            return
        if code not in (4, 5, 6):
            self.get_logger().error(
                f"Previous goal returned non-terminal status {code}; handoff remains blocked"
            )
            return
        self._release_goal_handoff(generation, result)

    def _release_goal_handoff(self, generation: int, result: str) -> None:
        with self.lock:
            if generation != self.goal_generation or not self.waiting_for_cancel:
                return
            self.waiting_for_cancel = False
            self.cancel_started_s = 0.0
            self.cancel_goal_handle = None
            self.cancel_attempt = 0
            goal = self.pending_goal if self.busy else None
            self.last_result = result
        self._publish_status()
        if goal is not None:
            if (
                self.active_goal_id == self.nurse_goal_id
                and self.nurse_scan_mode == "approach"
            ):
                self._send_next_nurse_viewpoint(generation)
            else:
                self._send_goal(goal, generation)

    def _send_goal(self, goal, generation: int) -> None:
        with self.lock:
            if generation != self.goal_generation or self.waiting_for_cancel:
                return
        if not self.nav_client.server_is_ready():
            with self.lock:
                if generation != self.goal_generation:
                    return
                self.pending_goal = goal
                self.nav_status = NAV_WAIT_PATH
                self.last_result = "waiting_for_nav2"
            self.get_logger().warn(
                "NavigateToPose server not ready; keeping goal queued",
                throttle_duration_sec=2.0,
            )
            return

        with self.lock:
            if generation != self.goal_generation:
                return
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
        if goal.goal_id == self.nurse_goal_id:
            request.behavior_tree = self.nurse_bt_xml
        self.get_logger().info(
            f"Dispatching {goal.name} -> map({target.pose.position.x:.2f}, "
            f"{target.pose.position.y:.2f}) request={self.active_request_id}"
        )
        future = self.nav_client.send_goal_async(
            request,
            feedback_callback=lambda feedback: self._feedback(feedback, generation),
        )
        future.add_done_callback(
            lambda completed: self._goal_accepted(completed, generation, goal)
        )

    def _pending_goal_tick(self) -> None:
        retry_cancel = None
        retry_nurse_cancel = None
        advance_nurse = False
        publish_nurse_timeout = False
        with self.lock:
            generation = self.goal_generation
            if self.waiting_for_cancel:
                elapsed = time.monotonic() - self.cancel_started_s
                if elapsed < self.goal_handoff_timeout_s:
                    return
                retry_cancel = self.cancel_goal_handle
            elif (
                self.busy
                and self.active_goal_id == self.nurse_goal_id
                and self.nurse_scan_mode
                in {
                    "stopping_for_entry",
                    "stopping_for_qr",
                    "skipping_viewpoint",
                }
                and self.active_goal_handle is not None
            ):
                elapsed = time.monotonic() - self.nurse_cancel_started_s
                if elapsed < self.goal_handoff_timeout_s:
                    return
                retry_nurse_cancel = self.active_goal_handle
            elif (
                self.busy
                and self.active_goal_id == self.nurse_goal_id
                and self.nurse_scan_mode == "dwell"
                and time.monotonic() >= self.nurse_dwell_deadline_s
            ):
                advance_nurse = True
            elif (
                self.busy
                and self.active_goal_id == self.nurse_goal_id
                and self.nurse_scan_mode == "moving_to_viewpoint"
                and self.nurse_action_started_s > 0.0
                and time.monotonic() - self.nurse_action_started_s
                >= self.nurse_scan_config.viewpoint_timeout_s
                and self.active_goal_handle is not None
            ):
                self.nurse_scan_mode = "skipping_viewpoint"
                self.nav_status = NAV_WAIT_PATH
                self.last_result = "nurse_viewpoint_timeout"
                retry_nurse_cancel = self.active_goal_handle
                publish_nurse_timeout = True
            goal = self.pending_goal if self.busy else None
        if retry_cancel is not None:
            self.get_logger().warn(
                "Previous goal is still not terminal; retrying cancellation "
                "while motion remains blocked"
            )
            self._request_previous_goal_cancel(retry_cancel, generation)
            return
        if retry_nurse_cancel is not None:
            if publish_nurse_timeout:
                self._publish_status()
            self._request_nurse_cancel(retry_nurse_cancel, generation)
            return
        if advance_nurse:
            self._send_next_nurse_viewpoint(generation)
            return
        if goal is not None and self.nav_client.server_is_ready():
            self._send_goal(goal, generation)

    def _feedback(self, feedback, generation: int) -> None:
        # Nav2 reports the remaining distance; keep the chassis lamp honest.
        started_following = False
        with self.lock:
            if generation != self.goal_generation:
                return
            if (
                self.active_goal_id == self.nurse_goal_id
                and self.nurse_scan_mode
                in {
                    "stopping_for_entry",
                    "stopping_for_qr",
                    "skipping_viewpoint",
                    "dwell",
                    "qr_hold",
                }
            ):
                return
            if self.nav_status != NAV_FOLLOWING:
                self.nav_status = NAV_FOLLOWING
                self.last_result = "following"
                started_following = True
        if started_following:
            self._publish_status()

    def _goal_accepted(self, future, generation: int, requested_goal) -> None:
        handle = future.result()
        with self.lock:
            current_generation = generation == self.goal_generation
        if not current_generation:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
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
                if (
                    generation == self.goal_generation
                    and self.busy
                    and requested_goal is not None
                    and self.rejection_retry_count
                    < self.startup_rejection_retry_limit
                ):
                    self.rejection_retry_count += 1
                    retry_count = self.rejection_retry_count
                    self.pending_goal = requested_goal
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
            self._finish(NAV_ERROR, "rejected", generation)
            return
        stop_for_qr = False
        with self.lock:
            if generation != self.goal_generation:
                handle.cancel_goal_async()
                return
            self.active_goal_handle = handle
            self.nav_status = NAV_WAIT_PATH
            if (
                self.active_goal_id == self.nurse_goal_id and self.nurse_qr_seen
            ):
                self.nurse_scan_mode = "stopping_for_qr"
                self.last_result = "nurse_qr_seen_stopping"
                stop_for_qr = True
            else:
                if (
                    self.active_goal_id == self.nurse_goal_id
                    and self.nurse_scan_mode == "moving_to_viewpoint"
                ):
                    self.nurse_action_started_s = time.monotonic()
                self.last_result = "accepted_waiting_feedback"
        self._publish_status()
        handle.get_result_async().add_done_callback(
            lambda completed: self._goal_finished(completed, generation)
        )
        if stop_for_qr:
            self._request_nurse_cancel(handle, generation)

    def _goal_finished(self, future, generation: int) -> None:
        with self.lock:
            if generation != self.goal_generation:
                return
            is_nurse = self.active_goal_id == self.nurse_goal_id
            nurse_mode = self.nurse_scan_mode
            nurse_qr_seen = self.nurse_qr_seen
            next_goal = self.nurse_next_goal
        status = future.result()
        code = int(getattr(status, "status", 0))
        # action_msgs/GoalStatus: 4 = succeeded, 5 = canceled, 6 = aborted.
        if is_nurse and nurse_qr_seen:
            self._hold_nurse_for_qr(generation)
            return
        if is_nurse and nurse_mode == "stopping_for_entry":
            with self.lock:
                if generation != self.goal_generation:
                    return
                self.active_goal_handle = None
                self.nurse_next_goal = None
            if next_goal is not None:
                self._queue_nurse_scan_goal(
                    next_goal, generation, "aligning_entry_toward_nurse_qr"
                )
            else:
                self._send_next_nurse_viewpoint(generation)
            return
        if is_nurse and nurse_mode == "skipping_viewpoint":
            with self.lock:
                if generation != self.goal_generation:
                    return
                self.active_goal_handle = None
            self._send_next_nurse_viewpoint(generation)
            return
        if is_nurse and nurse_mode == "moving_to_viewpoint":
            if code == 4:
                self._begin_nurse_dwell(generation, "nurse_scan_dwell")
            else:
                with self.lock:
                    if generation != self.goal_generation:
                        return
                    self.active_goal_handle = None
                self._send_next_nurse_viewpoint(generation)
            return
        if is_nurse and nurse_mode == "approach":
            if code == 4:
                self._begin_nurse_dwell(generation, "nurse_goal_scan_dwell")
            else:
                with self.lock:
                    if generation != self.goal_generation:
                        return
                    self.active_goal_handle = None
                self._send_next_nurse_viewpoint(generation)
            return
        if code == 4:
            self._finish(NAV_REACHED, "succeeded", generation)
            return
        if code == 5:
            self._finish(NAV_ERROR, "canceled", generation)
            return
        if self.attempt < self.retry_limit:
            self.attempt += 1
            with self.lock:
                if generation != self.goal_generation:
                    return
                self.nav_status = NAV_WAIT_PATH
            self.get_logger().warn(f"Nav2 failed (status {code}); retrying once")
            self._publish_status()
            self._send_goal(self.goals[self.active_goal_id], generation)
            return
        self._finish(NAV_ERROR, f"failed_status_{code}", generation)

    def _finish(self, nav_status: int, result: str, generation: int) -> None:
        with self.lock:
            if generation != self.goal_generation:
                return
            self.busy = False
            self.pending_goal = None
            self.active_goal_handle = None
            self.cancel_goal_handle = None
            self.waiting_for_cancel = False
            self.cancel_started_s = 0.0
            self.cancel_attempt = 0
            self._reset_nurse_scan_locked(False)
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
                "waiting_for_cancel": self.waiting_for_cancel,
                "cancel_attempt": self.cancel_attempt,
                "nurse_scan_mode": self.nurse_scan_mode,
                "nurse_viewpoint_index": self.nurse_viewpoint_index,
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
