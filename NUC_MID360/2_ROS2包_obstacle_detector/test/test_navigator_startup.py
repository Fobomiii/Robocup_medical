"""Exercise the real dispatcher with deterministic ROS/action stand-ins.

No DDS or hardware is available on Windows. Only the ROS transport boundary is
replaced; requests, timers and callbacks run MedicalNavigator's production code.
"""

import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from obstacle_detector.nav_protocol import (
    Frame, FrameParser, GOAL_NONE, GOAL_BED1, GOAL_BED3, GOAL_NURSE, GOAL_HOME,
    MSG_GOAL_REQUEST, MSG_NAV_STATUS, MSG_POSE, MSG_STP23L, MSG_WHEEL_ODOM,
    NAV_ERROR, NAV_FOLLOWING, NAV_IDLE, NAV_WAIT_PATH,
    TASK_NAV_BED1, TASK_NAV_BED3, TASK_NAV_HOME, TASK_NAV_NURSE,
    TASK_WAIT_BED1_START, TASK_WAIT_BED3_START,
    TASK_WAIT_START,
)
from obstacle_detector.stp23l_calibration import (
    CalibrationConfig, CalibrationEvent, OpsRangeCalibrator,
)


PACKAGE = Path(__file__).resolve().parents[1]


class Future:
    def __init__(self):
        self.callbacks = []
        self.value = None
        self.error = None
        self.complete = False

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.complete:
            callback(self)

    def result(self):
        if self.error:
            raise self.error
        return self.value

    def set_result(self, value):
        self.value, self.complete = value, True
        for callback in list(self.callbacks):
            callback(self)


class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = Future()
        self.cancel_goal_async = Mock(return_value=Future())

    def get_result_async(self):
        return self.result_future


class PoseStamped:
    def __init__(self):
        self.header = NS(frame_id="map", stamp=NS(sec=100, nanosec=0))
        self.pose = NS(position=NS(x=0.0, y=0.0, z=0.0),
                       orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))


class Odometry:
    def __init__(self):
        pose = PoseStamped()
        self.header = pose.header
        self.child_frame_id = ""
        self.pose = NS(pose=pose.pose, covariance=[0.0] * 36)
        self.twist = NS(twist=NS(linear=NS(x=0.0, y=0.0, z=0.0),
                                 angular=NS(x=0.0, y=0.0, z=0.0)),
                        covariance=[0.0] * 36)


class TransformStamped:
    def __init__(self):
        self.header = NS(frame_id="", stamp=None)
        self.child_frame_id = ""
        self.transform = NS(translation=NS(x=0.0, y=0.0, z=0.0),
                            rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0))


class Range:
    INFRARED = 1

    def __init__(self):
        self.header = NS(frame_id="", stamp=None)


class PathMessage:
    def __init__(self):
        self.header = NS(frame_id="map", stamp=NS(sec=100, nanosec=0))
        self.poses = []


class Float32MultiArray:
    def __init__(self):
        self.layout = NS(dim=[], data_offset=0)
        self.data = []


class MultiArrayDimension:
    def __init__(self, label="", size=0, stride=0):
        self.label = label
        self.size = size
        self.stride = stride


class ActionClient:
    def __init__(self, node, action, name):
        self.available = True
        self.requests = []

    def server_is_ready(self):
        return self.available

    def send_goal_async(self, request, **kwargs):
        future = Future()
        self.requests.append((request, future, kwargs))
        return future


class ServiceClient:
    def __init__(self):
        self.available = True
        self.requests = []

    def service_is_ready(self):
        return self.available

    def call_async(self, request):
        future = Future()
        self.requests.append(future)
        return future


class Node:
    def __init__(self, name):
        self.parameters = {}
        self.log = Mock()
        self.publishers = {}
        self.subscriptions = {}
        self.clients = {}

    def declare_parameter(self, name, value):
        self.parameters[name] = value
        return NS(value=value)

    def get_parameter(self, name):
        return NS(value=self.parameters[name])

    def get_logger(self):
        return self.log

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=100_000_000_000,
                                to_msg=lambda: NS(sec=100, nanosec=0)))

    def create_publisher(self, kind, topic, qos):
        publisher = Mock()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_timer(self, period, callback):
        return NS(period=period, callback=callback)

    def create_client(self, kind, service):
        client = ServiceClient()
        self.clients[service] = client
        return client


def load_ros_module(filename, module_name, source=None):
    modules = {}

    def module(name, **members):
        result = ModuleType(name)
        result.__dict__.update(members)
        modules[name] = result

    module("ament_index_python")
    module("ament_index_python.packages",
           get_package_share_directory=lambda name: str(PACKAGE))
    module("builtin_interfaces")
    module("builtin_interfaces.msg",
           Duration=lambda sec=0, nanosec=0: NS(sec=sec, nanosec=nanosec))
    module("geometry_msgs")
    module("geometry_msgs.msg", PoseStamped=PoseStamped,
           TransformStamped=TransformStamped, Twist=NS)
    module("nav_msgs")
    module("nav_msgs.msg", Path=PathMessage, Odometry=Odometry)
    module("sensor_msgs")
    module("sensor_msgs.msg", Range=Range)
    module("tf2_ros", StaticTransformBroadcaster=Mock, TransformBroadcaster=Mock)
    # No serial port or playback is opened: bridge tests exercise callbacks
    # and the real wire encoder, not the hardware-owning constructor.
    module("obstacle_detector.serial_transport", SerialTransport=Mock)
    module("nav2_msgs")
    module("nav2_msgs.msg", SpeedLimit=lambda: NS(
        header=NS(stamp=None), percentage=False, speed_limit=0.0))
    module("nav2_msgs.action", NavigateToPose=NS(Goal=lambda: NS()),
           ComputePathToPose=NS(Goal=lambda: NS()),
           SmoothPath=NS(Goal=lambda: NS()))
    module("lifecycle_msgs")
    module("lifecycle_msgs.srv", GetState=NS(Request=lambda: NS()))
    module("rclpy")
    module("rclpy.node", Node=Node)
    module("rclpy.action", ActionClient=ActionClient)
    module("rclpy.qos", QoSProfile=lambda **kwargs: NS(**kwargs),
           QoSDurabilityPolicy=NS(TRANSIENT_LOCAL=1),
           QoSReliabilityPolicy=NS(RELIABLE=1))
    module("std_msgs")
    module("std_msgs.msg", String=lambda: NS(data=""), Bool=lambda: NS(data=False),
           UInt8=lambda: NS(data=0), UInt32=lambda: NS(data=0),
           Float32MultiArray=Float32MultiArray,
           MultiArrayDimension=MultiArrayDimension)
    spec = importlib.util.spec_from_file_location(
        module_name, PACKAGE / "obstacle_detector" / filename)
    target = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        if source is None:
            spec.loader.exec_module(target)
        else:
            exec(compile(source, str(spec.origin), "exec"), target.__dict__)
    return target


def load_dispatcher(source=None):
    return load_ros_module("medical_navigator.py",
                           "obstacle_detector._startup_test_dispatcher", source)


DISPATCHER = load_dispatcher()
BRIDGE = load_ros_module("stm32_bridge.py", "obstacle_detector._startup_test_bridge")
CORNER = load_ros_module("corner_speed_limiter.py", "obstacle_detector._test_corner")
STARTUP_PROBE = load_ros_module(
    "startup_probe.py", "obstacle_detector._test_startup_probe"
)


def bridge_callback_fixture():
    node = BRIDGE.Stm32Bridge.__new__(BRIDGE.Stm32Bridge)
    Node.__init__(node, "stm32_bridge")
    node.pose = NS(task_state=TASK_WAIT_START, nav_status=NAV_WAIT_PATH)
    node.pose_ros = None
    node.raw_pose_ros = None
    node.pose_measurement_stamp_ns = 0
    node.ops_measurement_stamp_ns = 0
    node.hwt_measurement_stamp_ns = 0
    node.pose_timing_source = "unavailable"
    node.last_pose_s = 10.0
    node.pose_timeout_s = 0.5
    node.previous_ros_pose = None
    node.previous_pose_timing_source = None
    node.pose_twist = node.twist = (0.0, 0.0, 0.0)
    node.twist_source = "unavailable"
    node.twist_filter_alpha = 0.35
    node.max_axis_speed_m_s = 3.0
    node.wheel_odom = None
    node.last_wheel_odom_s = 0.0
    node.wheel_odom_timeout_s = 0.15
    node.requested_request_id, node.requested_goal_id = 1, GOAL_NURSE
    node.navigator_request_id, node.navigator_goal_id = 0, GOAL_NONE
    node.navigator_state, node.navigator_plan_ready = NAV_IDLE, False
    node.navigator_result = "waiting_navigator"
    node.last_goal_publish_s = 10.0
    node.last_navigator_status_s = 0.0
    node.navigator_status_timeout_s = 1.2
    node.navigator_following_s = 0.0
    node.goal_request_s = 9.0
    node.goal_handoff_hold_s, node.navigator_following_hold_s = 0.0, 0.0
    node.stm32_readiness = NS(ready=True)
    node.enforce_task_gate = True
    node.nurse_scan_soft_stop, node.stopped = False, True
    node.gate_release_limiter = Mock()
    node.goal_pub = Mock()
    node._send_velocity = Mock()
    node.transport = NS(connected=True, send=Mock(return_value=True))
    node.tx_seq = 0
    node.mission_epoch_pub = Mock()
    node.last_pose_tx_stamp_ms = None
    return node


class NavigatorStartupTest(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(DISPATCHER.time, "monotonic", return_value=10.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = DISPATCHER.MedicalNavigator()

    def task(self, state):
        self.node.subscriptions["/medical_nav/task_state"](NS(data=state))

    def ready(self):
        self.node._stm32_ready_changed(NS(data=True))

    def request(self, goal=GOAL_NURSE, request=1):
        self.node._goal_request(NS(data=json.dumps(
            {"goal_id": goal, "request_id": request})))

    def activate(self, except_node=None):
        self.node.lifecycle_poll_not_before_s = 0.0
        for _ in self.node.lifecycle_clients:
            self.node._poll_nav2_lifecycle()
            pending = self.node.lifecycle_query
            self.assertIsNotNone(pending)
            name, future, _ = pending
            state = 2 if name == except_node else 3
            future.set_result(NS(current_state=NS(id=state)))

    def test_lifecycle_queries_wait_until_action_servers_are_ready(self):
        self.node.nav_client.available = False
        self.node._poll_nav2_lifecycle()
        self.assertIsNone(self.node.lifecycle_query)
        self.assertTrue(all(
            not client.requests for client in self.node.lifecycle_clients.values()
        ))

        self.node.nav_client.available = True
        self.node._poll_nav2_lifecycle()
        self.assertIsNone(self.node.lifecycle_query)
        self.now.return_value = 12.1
        self.node._poll_nav2_lifecycle()
        self.assertIsNotNone(self.node.lifecycle_query)

    def path(self, goal=None, stamp=100):
        goal = goal or self.node.pending_goal
        path = PathMessage()
        path.header.stamp.sec = stamp
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = goal.y_mm / 1000, -goal.x_mm / 1000
        path.poses = [pose]
        return path

    def prepare(self, goal=GOAL_NURSE, state=TASK_WAIT_START):
        self.task(state)
        self.ready()
        self.activate()
        self.request(goal)

    def complete_preview(self, accepted=True, code=4, path=None):
        request, future, _ = self.node.preview_client.requests[-1]
        handle = Handle(accepted)
        future.set_result(handle)
        if accepted:
            preview_path = path if path is not None else self.path()
            handle.result_future.set_result(NS(
                status=code, result=NS(path=preview_path)
            ))
            if code == 4 and self.node._path_matches_goal(
                preview_path, self.node.pending_goal
            ):
                _, smooth_future, _ = self.node.preview_smoother_client.requests[-1]
                smooth_handle = Handle(accepted)
                smooth_future.set_result(smooth_handle)
                if accepted:
                    smooth_handle.result_future.set_result(NS(
                        status=code, result=NS(path=preview_path)
                    ))
        return handle

    def test_waiting_for_button_never_executes_navigate_action(self):
        self.ready()
        self.request()
        self.assertEqual(len(self.node.nav_client.requests), 0)

    def test_wait_states_only_plan_all_three_destinations(self):
        for state, goal in ((TASK_WAIT_START, GOAL_NURSE),
                            (TASK_WAIT_BED1_START, GOAL_BED1),
                            (TASK_WAIT_BED3_START, GOAL_BED3)):
            with self.subTest(state=state):
                self.node = DISPATCHER.MedicalNavigator()
                self.prepare(goal, state)
                self.assertEqual(len(self.node.nav_client.requests), 0)
                self.complete_preview()
                self.assertTrue(self.node.plan_ready)
                self.assertEqual(self.node.nav_status, NAV_FOLLOWING)

    def test_prestart_plan_is_smoothed_before_start_is_offered(self):
        self.prepare(GOAL_BED1, TASK_WAIT_BED1_START)
        preview_path = self.path()
        self.complete_preview(path=preview_path)
        request = self.node.preview_smoother_client.requests[-1][0]
        self.assertIs(request.path, preview_path)
        self.assertEqual(request.smoother_id, "constrained_smoother")
        self.assertTrue(request.check_for_collisions)
        self.assertEqual(request.max_smoothing_duration.sec, 0)
        self.assertEqual(request.max_smoothing_duration.nanosec, 450_000_000)
        self.assertEqual(self.node.last_result, "prestart_smoothed_path_ready")
        self.node.plan_pub.publish.assert_called_with(preview_path)

    def test_prestart_smoothing_collision_falls_back_to_raw_path(self):
        self.prepare(GOAL_BED1, TASK_WAIT_BED1_START)
        raw_path = self.path()
        planner_future = self.node.preview_client.requests[-1][1]
        planner_handle = Handle()
        planner_future.set_result(planner_handle)
        planner_handle.result_future.set_result(NS(
            status=4, result=NS(path=raw_path)
        ))

        smooth_future = self.node.preview_smoother_client.requests[-1][1]
        smooth_handle = Handle()
        smooth_future.set_result(smooth_handle)
        smooth_handle.result_future.set_result(NS(
            status=6, result=NS(path=PathMessage())
        ))

        self.assertTrue(self.node.plan_ready)
        self.assertEqual(self.node.nav_status, NAV_FOLLOWING)
        self.assertEqual(
            self.node.last_result,
            "prestart_raw_path_ready:smooth_failed_or_empty_path",
        )
        self.node.plan_pub.publish.assert_called_with(raw_path)

    def test_stale_smoothing_failure_cannot_ready_new_bed_route(self):
        self.prepare()
        old_raw_path = self.path()
        planner_future = self.node.preview_client.requests[-1][1]
        planner_handle = Handle()
        planner_future.set_result(planner_handle)
        planner_handle.result_future.set_result(NS(
            status=4, result=NS(path=old_raw_path)
        ))
        old_smooth_future = self.node.preview_smoother_client.requests[-1][1]

        self.task(TASK_WAIT_BED1_START)
        self.request(GOAL_BED1, request=2)
        old_smooth_handle = Handle(accepted=False)
        old_smooth_future.set_result(old_smooth_handle)

        self.assertFalse(self.node.plan_ready)
        self.assertEqual(self.node.active_goal_id, GOAL_BED1)
        self.complete_preview()
        self.assertTrue(self.node.plan_ready)

    def test_more_than_twenty_prestart_rejections_keep_retrying(self):
        self.prepare()
        for attempt in range(30):
            self.complete_preview(accepted=False)
            self.assertTrue(self.node.busy)
            self.assertNotEqual(self.node.nav_status, NAV_ERROR)
            self.now.return_value = 11.0 + attempt * 2.0
            self.activate()
            self.node._pending_goal_tick()
        self.complete_preview()
        self.assertTrue(self.node.plan_ready)

    def test_inactive_bt_navigator_does_not_offer_start(self):
        self.task(TASK_WAIT_START)
        self.ready()
        self.activate(except_node="bt_navigator")
        self.request()
        self.assertFalse(self.node.plan_ready)
        self.assertEqual(len(self.node.preview_client.requests), 0)
        self.assertIn("bt_navigator", self.node.last_result)

    def test_inactive_smoother_never_offers_start_and_recovers_when_active(self):
        self.task(TASK_WAIT_START)
        self.ready()
        self.activate(except_node="smoother_server")
        self.request()
        self.assertFalse(self.node.plan_ready)
        self.assertEqual(len(self.node.preview_client.requests), 0)
        self.assertIn("smoother_server", self.node.last_result)
        self.activate()
        self.node._pending_goal_tick()
        self.complete_preview()
        self.assertTrue(self.node.plan_ready)
        self.assertEqual(len(self.node.nav_client.requests), 0)

    def test_every_running_goal_sends_an_explicit_route_behavior_tree(self):
        for state, goal in ((TASK_NAV_NURSE, GOAL_NURSE),
                            (TASK_NAV_BED1, GOAL_BED1),
                            (TASK_NAV_BED3, GOAL_BED3),
                            (TASK_NAV_HOME, GOAL_HOME)):
            with self.subTest(goal=goal):
                self.node = DISPATCHER.MedicalNavigator()
                self.prepare(goal, state)
                request = self.node.nav_client.requests[-1][0]
                expected = (self.node.nurse_bt_xml if goal == GOAL_NURSE
                            else self.node.nav_bt_xml)
                self.assertEqual(request.behavior_tree, expected)
                self.node._publish_status()
                status = json.loads(self.node.status_pub.publish.call_args.args[0].data)
                self.assertEqual(status["behavior_tree"], expected)

    def test_button_starts_execution_without_waiting_for_new_goal_request(self):
        self.prepare()
        self.complete_preview()
        self.task(TASK_NAV_NURSE)
        self.assertEqual(len(self.node.nav_client.requests), 1)
        self.assertFalse(self.node.plan_ready)
        self.assertEqual(self.node.active_request_id, 1)

    def test_bed_start_state_then_new_generation_executes_once(self):
        self.prepare(GOAL_BED1, TASK_WAIT_BED1_START)
        self.complete_preview()
        self.task(TASK_NAV_BED1)
        self.request(GOAL_BED1, request=2)
        self.assertEqual(len(self.node.nav_client.requests), 1)
        self.assertEqual(self.node.active_request_id, 2)
        self.assertIsNone(self.node.pending_goal)

    def test_bed_new_generation_then_start_state_executes_once(self):
        self.prepare(GOAL_BED1, TASK_WAIT_BED1_START)
        self.complete_preview()
        self.request(GOAL_BED1, request=2)
        self.task(TASK_NAV_BED1)
        self.assertEqual(len(self.node.nav_client.requests), 1)
        self.assertEqual(self.node.active_request_id, 2)
        self.assertIsNone(self.node.pending_goal)

    def test_long_button_wait_does_not_advance_nurse_viewpoint(self):
        self.prepare()
        self.complete_preview()
        self.now.return_value = 120.0
        self.activate()
        self.node._pending_goal_tick()
        self.assertEqual(self.node.nurse_viewpoint_index, 1)
        self.assertEqual(len(self.node.nav_client.requests), 0)

    def test_prestart_scan_does_not_destroy_queued_route_before_stm32_ack(self):
        self.prepare()
        goal = self.node.pending_goal
        self.node._scan_result(NS(data='{"context": 1, "value": "13"}'))
        self.assertEqual(self.node.pending_goal, goal)
        self.complete_preview()
        self.assertTrue(self.node.plan_ready)

    def test_new_bed_task_waits_for_matching_goal_request(self):
        self.prepare()
        self.complete_preview()
        self.task(TASK_WAIT_BED1_START)
        self.assertFalse(self.node.plan_ready)
        self.request(GOAL_BED1, request=2)
        self.complete_preview()
        self.assertEqual(self.node.active_goal_id, GOAL_BED1)
        self.assertTrue(self.node.plan_ready)

    def test_old_nurse_result_cannot_ready_the_new_bed_route(self):
        self.prepare()
        old_path = self.path()
        old_future = self.node.preview_client.requests[-1][1]
        old_handle = Handle()
        old_future.set_result(old_handle)
        self.task(TASK_WAIT_BED1_START)
        self.request(GOAL_BED1, request=2)
        old_handle.result_future.set_result(NS(status=4, result=NS(path=old_path)))
        self.assertFalse(self.node.plan_ready)
        self.complete_preview()
        self.assertTrue(self.node.plan_ready)

    def test_preview_failed_empty_and_wrong_endpoint_never_offer_start(self):
        for path_kind in ("empty", "wrong_endpoint", "wrong_frame"):
            with self.subTest(kind=path_kind):
                self.node = DISPATCHER.MedicalNavigator()
                self.prepare()
                path = self.path()
                if path_kind == "empty":
                    path.poses = []
                elif path_kind == "wrong_endpoint":
                    path.poses[-1].pose.position.x += 1.0
                else:
                    path.header.frame_id = "odom"
                self.complete_preview(path=path)
                self.assertFalse(self.node.plan_ready)
                self.assertTrue(self.node.busy)

    def test_preview_timeout_retries_and_ignores_late_accept(self):
        self.prepare()
        old_future = self.node.preview_client.requests[-1][1]
        self.now.return_value = 16.0
        self.activate()
        self.node._pending_goal_tick()
        self.assertFalse(self.node.preview_inflight)
        handle = Handle()
        old_future.set_result(handle)
        handle.cancel_goal_async.assert_called_once()
        self.now.return_value = 17.5
        self.node._pending_goal_tick()
        self.assertEqual(len(self.node.preview_client.requests), 2)
        self.complete_preview()
        self.assertTrue(self.node.plan_ready)

    def test_lost_telemetry_preserves_preview_and_delays_new_requests(self):
        self.prepare()
        old_future = self.node.preview_client.requests[-1][1]
        self.node._stm32_ready_changed(NS(data=False))
        self.request(GOAL_BED1, request=2)
        self.assertEqual(self.node.deferred_goal_request, (GOAL_BED1, 2))
        handle = Handle()
        old_future.set_result(handle)
        self.assertFalse(self.node.plan_ready)
        self.assertTrue(self.node.busy)
        self.assertEqual(self.node.active_goal_id, GOAL_NURSE)
        self.task(TASK_WAIT_BED1_START)
        self.ready()
        self.complete_preview()
        self.assertTrue(self.node.plan_ready)

    def test_mission_epoch_change_clears_active_mission(self):
        self.prepare()
        self.node._mission_epoch_changed(NS(data=7))
        self.assertTrue(self.node.busy)
        self.node._mission_epoch_changed(NS(data=8))
        self.assertFalse(self.node.busy)
        self.assertEqual(self.node.active_goal_id, GOAL_NONE)
        self.assertEqual(self.node.last_result, "mission_epoch_changed")


    def start_bed(self):
        self.prepare(GOAL_BED1, TASK_WAIT_BED1_START)
        self.complete_preview()
        self.task(TASK_NAV_BED1)
        return self.node.nav_client.requests[-1][1]

    def test_path_arriving_before_action_accept_is_not_lost(self):
        accepted = self.start_bed()
        path = self.path(self.node.sent_goal)
        self.node._nav2_plan(path)
        self.assertFalse(self.node.plan_ready)
        accepted.set_result(Handle())
        self.assertTrue(self.node.plan_ready)

    def test_prestart_path_cannot_unlock_running_goal(self):
        accepted = self.start_bed()
        accepted.set_result(Handle())
        self.node._nav2_plan(self.path(self.node.sent_goal, stamp=99))
        self.assertFalse(self.node.plan_ready)
        self.node._nav2_plan(self.path(self.node.sent_goal))
        self.assertTrue(self.node.plan_ready)

    def test_path_for_other_goal_cannot_unlock_running_goal(self):
        accepted = self.start_bed()
        accepted.set_result(Handle())
        self.node._nav2_plan(self.path(self.node.goals[GOAL_BED3]))
        self.assertFalse(self.node.plan_ready)

    def test_stale_lifecycle_status_revokes_start_gate(self):
        self.prepare()
        self.complete_preview()
        self.now.return_value = 20.0
        self.node._pending_goal_tick()
        self.assertFalse(self.node.plan_ready)
        self.assertEqual(self.node.nav_status, NAV_WAIT_PATH)

    def test_goal_queued_before_telemetry_is_dispatched_when_ready(self):
        self.request(GOAL_BED1, request=5)
        self.task(TASK_WAIT_BED1_START)
        self.activate()
        self.ready()
        self.complete_preview()
        self.assertEqual(self.node.active_request_id, 5)
        self.assertTrue(self.node.plan_ready)

    def test_preview_result_exception_keeps_request_retryable(self):
        self.prepare()
        handle = Handle()
        self.node.preview_client.requests[-1][1].set_result(handle)
        handle.result_future.error = RuntimeError("temporary server restart")
        handle.result_future.set_result(None)
        self.assertTrue(self.node.busy)
        self.assertFalse(self.node.plan_ready)
        self.now.return_value = 12.0
        self.node._pending_goal_tick()
        self.complete_preview()
        self.assertTrue(self.node.plan_ready)

    def test_preview_result_request_exception_is_canceled_and_retried(self):
        self.prepare()
        handle = Handle()
        handle.get_result_async = Mock(side_effect=RuntimeError("server disappeared"))
        self.node.preview_client.requests[-1][1].set_result(handle)
        handle.cancel_goal_async.assert_called_once()
        self.assertTrue(self.node.busy)
        self.assertFalse(self.node.plan_ready)
        self.now.return_value = 12.0
        self.node._pending_goal_tick()
        self.complete_preview()
        self.assertTrue(self.node.plan_ready)

    def test_nav_action_send_exception_does_not_lose_pending_goal(self):
        self.prepare(GOAL_BED1, TASK_WAIT_BED1_START)
        self.complete_preview()
        original = self.node.nav_client.send_goal_async
        self.node.nav_client.send_goal_async = Mock(side_effect=RuntimeError("restart"))
        self.task(TASK_NAV_BED1)
        self.assertIsNotNone(self.node.pending_goal)
        self.assertFalse(self.node.plan_ready)
        self.node.nav_client.send_goal_async = original
        self.node._pending_goal_tick()
        self.assertEqual(len(self.node.nav_client.requests), 1)

    def test_nurse_six_second_timeout_begins_only_after_press_and_accept(self):
        self.prepare()
        self.complete_preview()
        self.now.return_value = 200.0
        self.task(TASK_NAV_NURSE)
        handle = Handle()
        self.node.nav_client.requests[-1][1].set_result(handle)
        self.assertEqual(self.node.nurse_action_started_s, 200.0)
        self.now.return_value = 205.0
        self.node._pending_goal_tick()
        handle.cancel_goal_async.assert_not_called()
        self.now.return_value = 206.5
        self.node._pending_goal_tick()
        handle.cancel_goal_async.assert_called_once()

    def test_repeated_stm32_request_does_not_reset_preview(self):
        self.prepare()
        token = self.node.preview_token
        self.request()
        self.assertEqual(self.node.preview_token, token)
        self.assertEqual(len(self.node.preview_client.requests), 1)
        self.complete_preview()
        self.request()
        self.assertTrue(self.node.plan_ready)

    def test_goal_before_first_task_sample_is_resumed_by_task_callback(self):
        self.ready()
        self.activate()
        self.request(GOAL_BED1, request=3)
        self.assertEqual(len(self.node.nav_client.requests), 0)
        self.assertEqual(len(self.node.preview_client.requests), 0)
        self.task(TASK_WAIT_BED1_START)
        self.complete_preview()
        self.assertTrue(self.node.plan_ready)

    def test_preview_status_reaches_mcu_but_cannot_move_before_button(self):
        self.prepare()
        self.complete_preview()
        bridge = bridge_callback_fixture()
        bridge._navigator_status(self.node.status_pub.publish.call_args.args[0])
        wire = bridge.transport.send.call_args.args[0]
        frames = FrameParser().feed(wire)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].msg_type, MSG_NAV_STATUS)
        self.assertEqual(struct.unpack(">HBB", frames[0].payload),
                         (1, GOAL_NURSE, NAV_FOLLOWING))
        bridge.pose.nav_status = NAV_FOLLOWING
        self.now.return_value = 10.4
        self.assertFalse(bridge._motion_is_authorized())

    def test_navigator_restart_recovers_from_bridge_replay(self):
        bridge = bridge_callback_fixture()
        bridge.navigator_request_id, bridge.navigator_goal_id = 1, GOAL_NURSE
        bridge.navigator_state, bridge.navigator_plan_ready = NAV_FOLLOWING, True
        bridge.last_navigator_status_s = 10.0
        self.now.return_value = 12.0
        bridge.last_pose_s = 12.0
        bridge.goal_pub.publish.side_effect = self.node._goal_request
        bridge._republish_goal_if_needed()
        self.task(TASK_WAIT_START)
        self.activate()
        self.ready()
        self.complete_preview()
        bridge._navigator_status(self.node.status_pub.publish.call_args.args[0])
        self.assertEqual(bridge.navigator_state, NAV_FOLLOWING)
        self.assertTrue(bridge.navigator_plan_ready)
        self.assertEqual(self.node.active_request_id, 1)
        self.assertEqual(len(self.node.nav_client.requests), 0)


class StartupProbeTest(unittest.TestCase):
    def ready_statuses(self):
        return {
            "navigator": {
                "startup_ready": True,
                "inactive_startup_nodes": [],
                "startup_actions": {
                    "navigate_to_pose": True,
                    "compute_path_to_pose": True,
                    "smooth_path": True,
                },
            },
            "bridge": {
                "serial": True,
                "pose": True,
                "wheel_telemetry": True,
                "stm32_ready": True,
                "wheel": {"online_mask": 15},
            },
            "scanner": {
                "camera": True,
                "camera_profile": "MJPG:1920x1200",
                "camera_resolution": [1920, 1200],
                "tele_camera": True,
                "tele_camera_profile": "MJPG:1920x1080",
                "tele_camera_resolution": [1920, 1080],
            },
            "health": {"connected": False},
        }

    def test_strict_probe_requires_all_four_motors_and_camera_profiles(self):
        statuses = self.ready_statuses()
        self.assertEqual(
            STARTUP_PROBE.readiness_reason(
                statuses, require_hardware=True,
                require_scanner=True, require_health=True,
            ),
            "ready",
        )
        statuses["bridge"]["wheel"]["online_mask"] = 7
        self.assertEqual(
            STARTUP_PROBE.readiness_reason(statuses, require_hardware=True),
            "wheel_online_mask_7",
        )

    def test_nav_only_probe_consumes_authoritative_navigator_state(self):
        statuses = self.ready_statuses()
        statuses.pop("bridge")
        statuses.pop("scanner")
        statuses.pop("health")
        self.assertEqual(STARTUP_PROBE.readiness_reason(statuses), "ready")
        statuses["navigator"]["startup_ready"] = False
        statuses["navigator"]["inactive_startup_nodes"] = ["planner_server"]
        self.assertEqual(
            STARTUP_PROBE.readiness_reason(statuses),
            "nav2_inactive:planner_server",
        )

class BridgeStartupRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(BRIDGE.time, "monotonic", return_value=12.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.bridge = bridge_callback_fixture()
        self.bridge.last_pose_s = 12.0

    def test_configured_handoff_still_requires_stable_matching_following(self):
        bridge = self.bridge
        bridge.goal_handoff_hold_s, bridge.navigator_following_hold_s = 0.25, 0.1
        bridge.pose.task_state, bridge.pose.nav_status = TASK_NAV_NURSE, NAV_FOLLOWING
        bridge.navigator_request_id, bridge.navigator_goal_id = 1, GOAL_NURSE
        bridge.navigator_state = NAV_FOLLOWING
        bridge.last_navigator_status_s = bridge.goal_request_s = 12.0
        bridge.navigator_following_s = 12.125
        self.now.return_value = 12.249
        self.assertFalse(bridge._motion_is_authorized())
        self.now.return_value = 12.25
        self.assertTrue(bridge._motion_is_authorized())

        # A late FOLLOWING still needs its own 0.10 s debounce.
        bridge.navigator_following_s = 12.25
        self.now.return_value = 12.349
        self.assertFalse(bridge._motion_is_authorized())
        self.now.return_value = 12.351
        self.assertTrue(bridge._motion_is_authorized())
        bridge.navigator_request_id = 2
        self.assertFalse(bridge._motion_is_authorized())

    def test_zero_handoff_opens_immediately_but_preserves_motion_gates(self):
        bridge = self.bridge
        bridge.pose.task_state, bridge.pose.nav_status = TASK_NAV_NURSE, NAV_FOLLOWING
        bridge.navigator_request_id, bridge.navigator_goal_id = 1, GOAL_NURSE
        bridge.navigator_state = NAV_FOLLOWING
        bridge.last_navigator_status_s = bridge.goal_request_s = 12.0
        bridge.navigator_following_s = 12.0
        # No time has elapsed since the request and first matching FOLLOWING.
        self.assertTrue(bridge._motion_is_authorized())

        for key, value in (
            ("last_pose_s", 10.0),
            ("last_navigator_status_s", 10.0),
            ("navigator_request_id", 2),
            ("navigator_goal_id", GOAL_BED1),
            ("navigator_state", NAV_WAIT_PATH),
            ("navigator_following_s", 0.0),
            ("goal_request_s", 0.0),
            ("stm32_readiness", NS(ready=False)),
            ("pose", NS(task_state=TASK_WAIT_START, nav_status=NAV_FOLLOWING)),
            ("pose", NS(task_state=TASK_NAV_NURSE, nav_status=NAV_WAIT_PATH)),
        ):
            with self.subTest(key=key, value=value):
                with patch.object(bridge, key, value):
                    self.assertFalse(bridge._motion_is_authorized())

    def test_late_subscriber_gets_current_goal(self):
        self.bridge._republish_goal_if_needed()
        message = self.bridge.goal_pub.publish.call_args.args[0]
        self.assertEqual(json.loads(message.data)["goal_id"], GOAL_NURSE)
        self.assertEqual(json.loads(message.data)["request_id"], 1)
        self.assertEqual(self.bridge.last_goal_publish_s, 12.0)
        self.bridge._republish_goal_if_needed()
        self.bridge.goal_pub.publish.assert_called_once()

    def test_no_replay_if_navigator_has_matching_fresh_request(self):
        self.bridge.navigator_request_id, self.bridge.navigator_goal_id = 1, GOAL_NURSE
        self.bridge.last_navigator_status_s = 12.0
        self.bridge._republish_goal_if_needed()
        self.bridge.goal_pub.publish.assert_not_called()

    def test_no_replay_of_cleared_mission_or_stale_pose_or_running_task(self):
        for key, value in (("requested_goal_id", GOAL_NONE),
                           ("last_pose_s", 10.0)):
            with self.subTest(key=key):
                bridge = bridge_callback_fixture()
                bridge.last_pose_s = 12.0
                setattr(bridge, key, value)
                bridge._republish_goal_if_needed()
                bridge.goal_pub.publish.assert_not_called()
        self.bridge.pose.task_state = TASK_NAV_NURSE
        self.bridge._republish_goal_if_needed()
        self.bridge.goal_pub.publish.assert_not_called()

    def test_changed_mcu_goal_clears_old_gate_before_publishing(self):
        self.bridge.navigator_request_id, self.bridge.navigator_goal_id = 1, GOAL_NURSE
        self.bridge.navigator_state, self.bridge.navigator_plan_ready = NAV_FOLLOWING, True
        frame = Frame(MSG_GOAL_REQUEST, 1, struct.pack(">HB", 2, GOAL_BED1))
        self.bridge._handle_goal_request(frame)
        self.assertEqual(self.bridge.requested_goal_id, GOAL_BED1)
        self.assertEqual(self.bridge.navigator_state, NAV_WAIT_PATH)
        self.assertFalse(self.bridge.navigator_plan_ready)
        self.assertEqual(self.bridge.navigator_result, "waiting_navigator")
        self.bridge._send_velocity.assert_called_once_with(
            0.0, 0.0, 0.0, source="goal_handoff")

    def test_plan_refresh_failure_does_not_clear_mcu_request(self):
        bridge = self.bridge
        bridge.pose.nav_status = NAV_FOLLOWING
        bridge.previous_ros_pose = None
        bridge._as_ros = Mock(return_value=(0.0, 0.0, 0.0))
        bridge._wheel_odom_is_fresh = Mock(return_value=False)
        bridge.task_state_pub = Mock()
        bridge.pending_scan = None
        frame = Frame(MSG_POSE, 1, struct.pack(
            ">iihBBHB", 0, 0, 0, TASK_WAIT_START, NAV_WAIT_PATH, 0, 0))
        bridge._handle_pose(frame)
        self.assertEqual(bridge.requested_request_id, 1)
        self.assertEqual(bridge.requested_goal_id, GOAL_NURSE)
        self.assertEqual(bridge.pose_twist, (0.0, 0.0, 0.0))

    def test_real_bridge_readiness_recovers_in_all_start_wait_states(self):
        for state in (TASK_WAIT_START, TASK_WAIT_BED1_START, TASK_WAIT_BED3_START):
            with self.subTest(state=state):
                bridge = bridge_callback_fixture()
                bridge.pose.task_state = state
                bridge.wheel_odom = NS(forward_mm_s=0, left_mm_s=0,
                                       yaw_ccw_cdeg_s=0)
                bridge.stm32_telemetry_timeout_s = 0.25
                bridge.stm32_readiness = BRIDGE.Stm32ReadinessGate(0.75)
                bridge.last_stm32_gate_state = "waiting_telemetry"
                bridge.stm32_ready_pub = Mock()
                self.now.return_value = 12.0
                bridge.last_pose_s = bridge.last_wheel_odom_s = 12.0
                bridge._update_stm32_readiness()
                self.assertFalse(bridge.stm32_readiness.ready)
                self.now.return_value = 12.76
                bridge.last_pose_s = bridge.last_wheel_odom_s = 12.76
                bridge._update_stm32_readiness()
                self.assertTrue(bridge.stm32_readiness.ready)
                self.assertTrue(bridge.stm32_ready_pub.publish.call_args.args[0].data)

    def test_unplanned_or_mismatched_following_never_opens_mcu_gate(self):
        self.bridge._navigator_status(NS(data=json.dumps(
            {"request_id": 1, "goal_id": GOAL_NURSE, "state": NAV_FOLLOWING,
             "plan_ready": False, "result": "accepted_waiting_feedback"})))
        self.assertEqual(self.bridge.navigator_state, NAV_WAIT_PATH)
        payload = FrameParser().feed(self.bridge.transport.send.call_args.args[0])[0].payload
        self.assertEqual(struct.unpack(">HBB", payload)[2], NAV_WAIT_PATH)
        self.bridge.transport.send.reset_mock()
        self.bridge._navigator_status(NS(data=json.dumps(
            {"request_id": 2, "goal_id": GOAL_BED1, "state": NAV_FOLLOWING,
             "plan_ready": True})))
        self.bridge.transport.send.assert_not_called()
        self.assertEqual(self.bridge.navigator_state, NAV_WAIT_PATH)


class BridgeDirectOdometryTest(unittest.TestCase):
    """Exercise actual pose/wheel/calibration callbacks and odom/TF output."""

    def setUp(self):
        self.clock = patch.object(BRIDGE.time, "monotonic", return_value=12.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = bridge_callback_fixture()
        node = self.node
        node.pose = NS(x_mm=0, y_mm=0, yaw_cdeg=0,
                       task_state=TASK_NAV_BED1, nav_status=NAV_FOLLOWING)
        node.last_pose_s = 12.0
        node.yaw_sign = -1.0
        node.odom_frame, node.base_frame = "odom", "base_link"
        node.calibrator = OpsRangeCalibrator(
            CalibrationConfig(False, 144.3, 7, 40.0, 200.0, 8.0, 400.0, ()))
        node.sensor_radius_m = 0.1443
        node.ops_position_variance = 0.02**2
        node.wheel_twist_variances = (0.08**2, 0.16**2, 0.12**2)
        node.tf_broadcaster = Mock()
        node.odom_pub, node.pose_pub, node.raw_pose_pub = Mock(), Mock(), Mock()
        node.wheel_odom_pub, node.task_state_pub = Mock(), Mock()
        node.range_pubs = {sensor: Mock() for sensor in "abc"}
        node.stp23l_pub, node.calibration_pub = Mock(), Mock()
        node.heading_hold_controller, node.nurse_scan_stop_detector = Mock(), Mock()
        node.hwt_yaw_rate_rad_s, node.last_hwt_yaw_rate_s = 0.0, 0.0
        node.pending_scan = None
        node.mission_epoch = 7
        node.tts_results = {}

    def wheel(self, forward=2000, left=-1500, yaw=3000, mask=0x0F):
        self.node._handle_wheel_odom(Frame(MSG_WHEEL_ODOM, 1, struct.pack(
            ">hhhBH", forward, left, yaw, mask, 1200)))

    def pose(self, x=0, y=0, yaw=0, task=TASK_NAV_BED1):
        self.node._handle_pose(Frame(MSG_POSE, 1, struct.pack(
            ">iihBBHB", x, y, yaw, task, NAV_FOLLOWING, 0, 0)))

    def published_odom(self):
        self.node._publish_pose()
        return self.node.odom_pub.publish.call_args.args[0]

    def assert_twist(self, odom, expected):
        actual = (odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                  odom.twist.twist.angular.z)
        for value, target in zip(actual, expected):
            self.assertAlmostEqual(value, target)

    def test_pose_tf_and_odom_use_corrected_ops_and_direct_hwt(self):
        node = self.node
        node.calibrator.offset_x_mm, node.calibrator.offset_y_mm = 100.0, -50.0
        self.pose(x=1000, y=2000, yaw=9000)
        self.wheel()
        odom = self.published_odom()
        pose = node.pose_pub.publish.call_args.args[0].pose
        transform = node.tf_broadcaster.sendTransform.call_args.args[0].transform
        for position in (odom.pose.pose.position, pose.position, transform.translation):
            self.assertAlmostEqual(position.x, 1.95)
            self.assertAlmostEqual(position.y, -1.1)
        for orientation in (odom.pose.pose.orientation, pose.orientation, transform.rotation):
            self.assertAlmostEqual(orientation.z, math.sin(-math.pi / 4.0))
            self.assertAlmostEqual(orientation.w, math.cos(-math.pi / 4.0))
        raw = node.raw_pose_pub.publish.call_args.args[0].pose.position
        self.assertEqual((raw.x, raw.y), (2.0, -1.0))
        # Wheel frames and repeated publication must not integrate position.
        self.now.return_value = 12.1
        self.wheel(forward=-2500, left=3000, yaw=-4000)
        odom = self.published_odom()
        self.assertEqual((odom.pose.pose.position.x, odom.pose.pose.position.y),
                         (1.95, -1.1))

    def test_extended_pose_uses_sensor_time_and_aligns_ops_to_hwt(self):
        self.wheel(forward=2000, left=0, yaw=0)
        payload = struct.pack(
            ">iihBBHBIII",
            0, 0, 0, TASK_NAV_BED1, NAV_FOLLOWING, 0, 0,
            1000, 1050, 1100,
        )
        self.node._handle_pose(Frame(MSG_POSE, 1, payload))

        odom = self.published_odom()

        self.assertAlmostEqual(odom.pose.pose.position.x, 0.1, places=5)
        self.assertEqual((odom.header.stamp.sec, odom.header.stamp.nanosec),
                         (99, 950_000_000))
        self.assertEqual(self.node.pose_timing_source, "stm32_sample_time")

    def test_valid_wheel_velocity_is_direct_and_changes_without_filter_delay(self):
        for forward, left, yaw in ((2000, -1500, 3000), (-712, 238, -1500)):
            self.wheel(forward, left, yaw)
            expected = (forward / 1000.0, left / 1000.0, math.radians(yaw / 100.0))
            self.assert_twist(self.published_odom(), expected)
            self.assert_twist(self.node.wheel_odom_pub.publish.call_args.args[0], expected)
            self.assertEqual(self.node.twist_source, "wheel_direct")

    def test_wheel_timeout_and_offline_mask_use_independent_pose_difference(self):
        node = self.node
        node.previous_ros_pose = (0.0, 0.0, 0.0, 11.98)
        self.wheel()
        self.pose(y=20)
        self.assertAlmostEqual(node.pose_twist[0], 0.35)
        self.assertEqual(node.pose_twist[1], 0.0)
        self.assert_twist(self.published_odom(), (2.0, -1.5, math.pi / 6.0))
        # Pose stays fresh while the wheel frame expires. No old wheel speed
        # may remain in the fallback or its filter history.
        self.now.return_value = 12.16
        self.assert_twist(self.published_odom(), (0.35, 0.0, 0.0))
        self.assertEqual(node.twist_source, "ops_hwt_difference")
        self.wheel(mask=0x07)
        self.assert_twist(self.published_odom(), (0.35, 0.0, 0.0))
        self.assertFalse(node._wheel_odom_is_fresh())
        self.now.return_value = 12.7
        node._update_measured_twist()
        self.assertEqual(node.twist, (0.0, 0.0, 0.0))
        self.assertEqual(node.twist_source, "unavailable")

    def test_pose_difference_accepts_legal_fast_diagonal_but_rejects_reset_jump(self):
        node = self.node
        node.previous_ros_pose = (0.0, 0.0, 0.0, 11.98)
        node.twist_filter_alpha = 1.0
        self.pose(x=-60, y=60)
        self.assert_twist(self.published_odom(), (3.0, 3.0, 0.0))
        self.now.return_value = 12.02
        self.pose(x=-60, y=5060)
        self.assertEqual(node.pose_twist, (0.0, 0.0, 0.0))
        self.assert_twist(self.published_odom(), (0.0, 0.0, 0.0))

    def test_hwt_difference_wraps_at_180_degrees(self):
        node = self.node
        node.previous_ros_pose = (0.0, 0.0, math.radians(179.0), 11.98)
        node.twist_filter_alpha = 1.0
        self.pose(yaw=17900)
        self.assert_twist(self.published_odom(), (0.0, 0.0, math.radians(100.0)))

    def test_calibration_clears_difference_history_without_erasing_wheel_feedback(self):
        node = self.node
        self.pose(x=1000, y=2000)
        self.wheel()
        node.pose_twist = (1.0, 1.0, 0.0)

        def apply_offset(*args):
            node.calibrator.offset_x_mm, node.calibrator.offset_y_mm = 100.0, -50.0
            return CalibrationEvent("bed1", "applied", 7, True,
                                    100.0, -50.0, 1100.0, 1950.0)

        with patch.object(node.calibrator, "update", side_effect=apply_offset):
            node._handle_stp23l(Frame(MSG_STP23L, 1,
                                     struct.pack(">HHHB", 1000, 350, 1000, 7)))
        self.assertIsNone(node.previous_ros_pose)
        self.assertEqual(node.pose_twist, (0.0, 0.0, 0.0))
        self.assert_twist(self.published_odom(), (2.0, -1.5, math.pi / 6.0))
        self.now.return_value = 12.16
        self.pose(x=1000, y=2000)
        odom = self.published_odom()
        self.assertEqual((odom.pose.pose.position.x, odom.pose.pose.position.y),
                         (1.95, -1.1))
        self.assert_twist(odom, (0.0, 0.0, 0.0))

    def test_mission_reset_clears_old_difference_and_calibration(self):
        node = self.node
        node.pose.task_state = TASK_NAV_BED3
        node.pose_twist = node.twist = (2.0, 1.0, 0.5)
        node.previous_ros_pose = (4.0, -2.0, 1.0, 11.98)
        node.calibrator.offset_x_mm = 100.0
        self.pose(task=TASK_WAIT_START)
        self.assertEqual(node.pose_twist, (0.0, 0.0, 0.0))
        self.assertEqual(node.calibrator.offset_x_mm, 0.0)
        self.assertEqual(node.mission_epoch, 8)
        self.assertEqual(node.requested_goal_id, GOAL_NONE)
        self.assertFalse(node.navigator_plan_ready)


class CornerSpeedDisabledTest(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(CORNER.time, "monotonic", return_value=12.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = CORNER.CornerSpeedLimiter()

    def assert_no_limit(self):
        message = self.node.limit_pub.publish.call_args.args[0]
        self.assertEqual(message.speed_limit, 0.0)
        self.assertFalse(message.percentage)
        status = json.loads(self.node.status_pub.publish.call_args.args[0].data)
        self.assertEqual(status["source"], "disabled")
        self.assertFalse(status["enabled"])
        self.assertFalse(status["limited"])
        self.assertFalse(status["path_data_valid"])

    def test_disabled_by_default_without_pose_or_path(self):
        self.assertFalse(self.node.enabled)
        with patch.object(self.node, "_path_decisions") as decide:
            self.node._update()
        decide.assert_not_called()
        self.assert_no_limit()

    def test_reference_speed_shadow_is_empty_without_fresh_inputs(self):
        self.node._publish_reference_profile()

        profile = self.node.reference_profile_pub.publish.call_args.args[0]
        status = json.loads(
            self.node.reference_status_pub.publish.call_args.args[0].data
        )
        self.assertEqual(profile.data, [])
        self.assertFalse(status["valid"])
        self.assertEqual(status["reason"], "no_fresh_pose")
        self.assertTrue(status["shadow_mode"])
        self.assertFalse(status["affects_control"])
        self.node.limit_pub.publish.assert_not_called()

    def test_reference_speed_shadow_publishes_profile_without_control_output(self):
        node = self.node
        node.pose_x_m, node.pose_y_m, node.pose_received_s = 0.0, 0.0, 12.0
        node.odom_speed_m_s, node.odom_received_s = 1.0, 12.0
        node.raw_path = [(0.1 * index, 0.0) for index in range(61)]
        node.raw_path_received_s = 12.0

        node._publish_reference_profile()

        profile = node.reference_profile_pub.publish.call_args.args[0]
        status = json.loads(node.reference_status_pub.publish.call_args.args[0].data)
        self.assertGreater(len(profile.data), 8)
        self.assertEqual(len(profile.data) % 8, 0)
        self.assertTrue(status["valid"])
        self.assertEqual(status["source"], "raw_shadow_fallback")
        self.assertFalse(status["affects_control"])
        node.limit_pub.publish.assert_not_called()

    def test_disabled_clears_old_corner_cap_for_single_profile(self):
        node = self.node
        self.now.return_value = 12.0
        node.last_status_s = 0.0
        node.pose_x_m, node.pose_y_m, node.pose_received_s = 0.5, 0.0, 12.0
        node.raw_path = [(0.1 * i, 0.0) for i in range(11)]
        node.raw_path.extend((1.0, 0.1 * i) for i in range(1, 21))
        node.raw_path_received_s = 12.0
        node.smoothed_path = node.raw_path
        node.smoothed_path_received_s = 12.0
        node.enabled = True
        node._update()
        old_limit = node.limit_pub.publish.call_args.args[0].speed_limit
        self.assertGreater(old_limit, 0.0)
        self.assertLess(old_limit, node.max_speed_m_s)

        node.enabled = False
        with patch.object(node, "_path_decisions") as decide:
            self.now.return_value = 12.25
            node._update()
            self.assert_no_limit()
            # Keep clearing the cap even if pose/path become unavailable.
            node.pose_x_m = node.pose_y_m = None
            node.raw_path = node.smoothed_path = None
            self.now.return_value = 12.5
            node._update()
            self.assert_no_limit()
        decide.assert_not_called()


if __name__ == "__main__":
    unittest.main()
