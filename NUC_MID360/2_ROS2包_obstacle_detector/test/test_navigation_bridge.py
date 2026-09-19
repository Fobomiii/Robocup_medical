#!/usr/bin/env python3
"""Pseudo-terminal integration test for the ROS navigation serial bridge."""

import os
import select
import struct
import threading
import time
import unittest

import numpy as np
import rclpy

from obstacle_detector.nav_protocol import (
    FrameParser,
    GOAL_BED3,
    MSG_GOAL_REQUEST,
    MSG_PATH_BEGIN,
    MSG_PATH_CANCEL,
    MSG_PATH_COMMIT,
    MSG_POSE,
    MSG_WAYPOINT,
    encode_frame,
)
from obstacle_detector.navigation_node import MedicalNavigationNode, PING_FRAME, PONG_FRAME


class NavigationBridgeTest(unittest.TestCase):
    @staticmethod
    def _read_until(master_fd, predicate, timeout=2.0):
        raw = bytearray()
        parser = FrameParser()
        frames = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            readable, _, _ = select.select([master_fd], [], [], 0.05)
            if readable:
                chunk = os.read(master_fd, 4096)
                raw.extend(chunk)
                frames.extend(parser.feed(chunk))
            if predicate(frames):
                break
        return raw, frames

    @staticmethod
    def _start_node(slave_name):
        rclpy.init(
            args=[
                "--ros-args",
                "-p", f"serial_port:={slave_name}",
                "-p", "planner_enabled:=true",
                "-p", "replan_cooldown_s:=0.0",
            ]
        )
        node = MedicalNavigationNode()
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        deadline = time.monotonic() + 2.0
        while not node.transport.connected and time.monotonic() < deadline:
            time.sleep(0.01)
        return node, spin_thread

    @staticmethod
    def _stop_node(node, spin_thread, master_fd):
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        os.close(master_fd)

    def test_fixed_routes_do_not_cross_inflated_fixtures(self):
        from obstacle_detector.fixed_routes import FixedRouteMap

        config = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), "config", "field_map.yaml"
        )
        route_map = FixedRouteMap(config)
        inflation_mm = 300.0
        for (source_name, _), waypoints in route_map.routes.items():
            source = route_map.goals_by_name[source_name]
            points = [(source.x_mm, source.y_mm)] + [
                (point.x_mm, point.y_mm) for point in waypoints
            ]
            for fixture in route_map.raw.get("fixtures", []):
                xmin = fixture["x_mm"] - fixture["width_mm"] / 2.0 - inflation_mm
                xmax = fixture["x_mm"] + fixture["width_mm"] / 2.0 + inflation_mm
                ymin = fixture["y_mm"] - fixture["height_mm"] / 2.0 - inflation_mm
                ymax = fixture["y_mm"] + fixture["height_mm"] / 2.0 + inflation_mm
                for start, end in zip(points, points[1:]):
                    samples = 100
                    inside = any(
                        xmin < start[0] + (end[0] - start[0]) * step / samples < xmax
                        and ymin < start[1] + (end[1] - start[1]) * step / samples < ymax
                        for step in range(samples + 1)
                    )
                    self.assertFalse(
                        inside,
                        f"route {source_name} segment {start}->{end} crosses {fixture['name']}",
                    )

    def test_pose_goal_returns_complete_path(self):
        master_fd, slave_fd = os.openpty()
        slave_name = os.ttyname(slave_fd)
        os.close(slave_fd)
        node, spin_thread = self._start_node(slave_name)

        try:
            self.assertTrue(node.transport.connected)

            pose_payload = struct.pack(">iihBBHB", -2100, 5800, 0, 0, 0, 0, 0)
            goal_payload = struct.pack(">HB", 1, GOAL_BED3)
            os.write(
                master_fd,
                PING_FRAME
                + encode_frame(MSG_POSE, 1, pose_payload)
                + encode_frame(MSG_GOAL_REQUEST, 2, goal_payload),
            )

            raw, frames = self._read_until(
                master_fd,
                lambda received: any(
                    frame.msg_type == MSG_PATH_COMMIT for frame in received
                ),
            )

            types = [frame.msg_type for frame in frames]
            self.assertIn(PONG_FRAME, raw)
            self.assertEqual(types.count(MSG_PATH_BEGIN), 1)
            self.assertGreaterEqual(types.count(MSG_WAYPOINT), 1)
            self.assertLessEqual(types.count(MSG_WAYPOINT), 16)
            self.assertEqual(types.count(MSG_PATH_COMMIT), 1)
        finally:
            self._stop_node(node, spin_thread, master_fd)

    def test_confirmed_obstacle_cancels_and_replaces_path(self):
        master_fd, slave_fd = os.openpty()
        slave_name = os.ttyname(slave_fd)
        os.close(slave_fd)
        node, spin_thread = self._start_node(slave_name)

        try:
            pose_payload = struct.pack(">iihBBHB", 0, 0, 0, 0, 0, 0, 0)
            goal_payload = struct.pack(">HB", 7, GOAL_BED3)
            os.write(
                master_fd,
                encode_frame(MSG_POSE, 1, pose_payload)
                + encode_frame(MSG_GOAL_REQUEST, 2, goal_payload),
            )
            _, initial_frames = self._read_until(
                master_fd,
                lambda received: any(
                    frame.msg_type == MSG_PATH_COMMIT for frame in received
                ),
            )
            initial_begin = next(
                frame for frame in initial_frames if frame.msg_type == MSG_PATH_BEGIN
            )
            initial_path_id = struct.unpack(">H", initial_begin.payload[:2])[0]

            offsets = np.linspace(-0.035, 0.035, 9)
            points = np.column_stack(
                (
                    0.9 + offsets,
                    -0.5 + offsets[::-1],
                    np.full_like(offsets, 0.2),
                )
            )
            for index in range(3):
                node.mapper.update(points, 0.0, 0.0, 0.0, 1.0 + index * 0.1)
            self.assertEqual(len(node.mapper.obstacles()), 1)
            self.assertTrue(
                node.planner.path_blocked(
                    (0.0, 0.0), node.current_path, node.mapper.obstacles()
                )
            )

            node.replan_pending = True
            node.last_replan_s = 0.0
            node._planner_tick()
            _, replan_frames = self._read_until(
                master_fd,
                lambda received: any(
                    frame.msg_type == MSG_PATH_COMMIT for frame in received
                ),
            )
            types = [frame.msg_type for frame in replan_frames]
            self.assertIn(MSG_PATH_CANCEL, types)
            self.assertIn(MSG_PATH_BEGIN, types)
            self.assertLess(types.index(MSG_PATH_CANCEL), types.index(MSG_PATH_BEGIN))
            new_begin = next(
                frame for frame in replan_frames if frame.msg_type == MSG_PATH_BEGIN
            )
            new_path_id = struct.unpack(">H", new_begin.payload[:2])[0]
            self.assertNotEqual(initial_path_id, new_path_id)
            self.assertEqual(node.planner_state, "replanned")
            self.assertEqual(node.replan_count, 1)
            self.assertFalse(
                node.planner.path_blocked(
                    (0.0, 0.0), node.current_path, node.mapper.obstacles()
                )
            )
        finally:
            self._stop_node(node, spin_thread, master_fd)


if __name__ == "__main__":
    unittest.main()
