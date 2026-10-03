"""Exercise the limiter callbacks without requiring ROS on the test host."""

import importlib.util
import json
import math
from pathlib import Path
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


class Message(SimpleNamespace):
    def __init__(self):
        super().__init__(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="map")
        )


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self, name):
        self.subscriptions = []

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=default)

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions.append(topic)

    def create_timer(self, *args):
        pass

    def get_logger(self):
        return SimpleNamespace(info=lambda *args: None)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))


def load_limiter():
    modules = {}
    for name, entries in {
        "geometry_msgs.msg": {"PoseStamped": Message},
        "nav_msgs.msg": {"Path": Message},
        "nav2_msgs.msg": {"SpeedLimit": Message},
        "std_msgs.msg": {"String": Message},
        "rclpy": {},
        "rclpy.node": {"Node": Node},
        "rclpy.qos": {
            "QoSProfile": SimpleNamespace,
            "QoSDurabilityPolicy": SimpleNamespace(TRANSIENT_LOCAL=1),
            "QoSReliabilityPolicy": SimpleNamespace(RELIABLE=1),
        },
    }.items():
        module = ModuleType(name)
        module.__dict__.update(entries)
        modules[name] = module
    source = Path(__file__).resolve().parents[1] / "obstacle_detector" / "corner_speed_limiter.py"
    spec = importlib.util.spec_from_file_location("obstacle_detector._limiter_under_test", source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", modules):
        spec.loader.exec_module(module)
    return module.CornerSpeedLimiter


CornerSpeedLimiter = load_limiter()


def path_message(points, frame="map"):
    message = Message()
    message.header.frame_id = frame
    message.header.stamp = SimpleNamespace(sec=42, nanosec=123)
    message.poses = [
        SimpleNamespace(
            header=SimpleNamespace(frame_id=frame),
            pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)),
        )
        for x, y in points
    ]
    return message


class CornerSpeedLimiterTest(unittest.TestCase):
    def setUp(self):
        self.node = CornerSpeedLimiter()
        self.node.pose_x_m = 0.0
        self.node.pose_y_m = 0.0
        self.node.pose_yaw_rad = 0.0
        self.node.pose_received_s = time.monotonic()

    def test_only_selected_execution_path_is_subscribed(self):
        self.assertIn("/plan", self.node.subscriptions)
        self.assertIn("/plan", self.node.subscriptions)
        self.assertNotIn("/plan_smoothed", self.node.subscriptions)
        self.assertNotIn("/transformed_global_plan", self.node.subscriptions)

    def test_other_safe_path_cannot_release_executing_blind_path(self):
        self.node.raw_path = [(0.0, 0.0), (3.0, 0.0)]
        self.node._execution_path(path_message([(0.0, 0.0), (2.0, 2.0)]))
        self.assertAlmostEqual(self.node.limit_pub.messages[-1].speed_limit, 0.70)
        status = json.loads(self.node.status_pub.messages[-1].data)
        self.assertEqual(status["source"], "execution_path")
        self.assertEqual(status["path_version"], "42.000000123")
        self.assertTrue(status["blind_zone_limited"])

    def test_real_raw_fallback_replaces_rejected_smoothed_path(self):
        self.node._execution_path(path_message([(0.0, 0.0), (2.0, 2.0)]))
        fallback = [(0.0, 0.0), (1.2, 0.0), (1.2, 1.2)]
        self.node._execution_path(path_message(fallback))
        decisions = self.node._path_decisions(time.monotonic())
        self.assertEqual(self.node.execution_path, fallback)
        self.assertEqual(decisions[0][2].overlap_distance_m, 0.0)
        self.assertEqual(decisions[0][2].speed_limit_m_s, 3.0)

    def test_safe_straight_path_keeps_full_speed(self):
        self.node._execution_path(path_message([(0.0, 0.0), (3.0, 0.0)]))
        self.assertEqual(self.node.limit_pub.messages[-1].speed_limit, 0.0)

    def test_missing_or_expired_path_keeps_low_speed_protection(self):
        self.node._update()
        self.assertEqual(self.node.limit_pub.messages[-1].speed_limit, 0.70)
        self.node._execution_path(path_message([(0.0, 0.0), (3.0, 0.0)]))
        self.node.execution_path_received_s -= 10.0
        self.node._update()
        self.assertEqual(self.node.limit_pub.messages[-1].speed_limit, 0.70)

    def test_missing_pose_cannot_release_blind_protection(self):
        self.node.pose_received_s -= 10.0
        self.node._execution_path(path_message([(0.0, 0.0), (3.0, 0.0)]))
        self.assertEqual(self.node.limit_pub.messages[-1].speed_limit, 0.70)

    def test_invalid_pose_cannot_release_blind_protection(self):
        self.node.pose_yaw_rad = math.nan
        self.node._execution_path(path_message([(0.0, 0.0), (3.0, 0.0)]))
        self.assertEqual(self.node.limit_pub.messages[-1].speed_limit, 0.70)

    def test_invalid_frame_empty_or_nonfinite_path_retains_protection(self):
        for message in (
            path_message([(0.0, 0.0), (3.0, 0.0)], "base_link"),
            path_message([]),
            path_message([(0.0, 0.0), (math.nan, 0.0), (3.0, 0.0)]),
        ):
            self.node._execution_path(message)
            self.assertIsNone(self.node.execution_path)
            self.assertEqual(self.node.limit_pub.messages[-1].speed_limit, 0.70)

    def test_mixed_pose_frame_cannot_be_silently_connected(self):
        message = path_message([(0.0, 0.0), (3.0, 0.0)])
        message.poses[1].header.frame_id = "odom"
        self.node._execution_path(message)
        self.assertIsNone(self.node.execution_path)

    def test_all_four_diagonal_directions_are_symmetric(self):
        for x_sign in (-1.0, 1.0):
            for y_sign in (-1.0, 1.0):
                self.node._execution_path(
                    path_message([(0.0, 0.0), (2.0 * x_sign, 2.0 * y_sign)])
                )
                self.assertEqual(self.node.limit_pub.messages[-1].speed_limit, 0.70)

    def test_runtime_rechecks_actual_yaw_on_same_execution_path(self):
        self.node._execution_path(path_message([(0.0, 0.0), (3.0, 0.0)]))
        self.node.pose_yaw_rad = math.radians(45.0)
        self.node._update()
        self.assertEqual(self.node.limit_pub.messages[-1].speed_limit, 0.70)

    def test_explicit_blind_guard_disable_is_preserved(self):
        self.node.blind_zone_enabled = False
        self.node._update()
        self.assertEqual(self.node.limit_pub.messages[-1].speed_limit, 0.0)


if __name__ == "__main__":
    unittest.main()
