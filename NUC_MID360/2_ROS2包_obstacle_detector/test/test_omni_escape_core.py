#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from obstacle_detector.omni_escape_core import (  # noqa: E402
    BlockedIntentLatch,
    StallDetector,
    choose_escape_direction,
    direction_is_swept_clear,
    path_length,
    supported_clearance,
)


class BlockedIntentLatchTest(unittest.TestCase):
    def setUp(self):
        self.latch = BlockedIntentLatch(
            hold_s=6.0,
            minimum_upstream_speed_m_s=0.18,
            maximum_safe_speed_m_s=0.03,
        )

    def test_collision_suppressed_command_is_retained_through_recovery(self):
        self.assertTrue(
            self.latch.update(
                now_s=10.0,
                upstream_x_m_s=0.30,
                upstream_y_m_s=0.0,
                safe_speed_m_s=0.0,
            )
        )
        self.assertEqual(self.latch.get(14.0), (0.30, 0.0))
        self.assertIsNone(self.latch.get(16.01))

    def test_normal_stop_without_prior_motion_is_not_latched(self):
        self.assertFalse(
            self.latch.update(
                now_s=20.0,
                upstream_x_m_s=0.0,
                upstream_y_m_s=0.0,
                safe_speed_m_s=0.0,
            )
        )
        self.assertIsNone(self.latch.get(20.1))

    def test_nonzero_safe_output_is_not_a_collision_block(self):
        self.assertFalse(
            self.latch.update(
                now_s=30.0,
                upstream_x_m_s=0.30,
                upstream_y_m_s=0.0,
                safe_speed_m_s=0.20,
            )
        )


class StallDetectorTest(unittest.TestCase):
    def setUp(self):
        self.detector = StallDetector(
            duration_s=4.0,
            minimum_upstream_speed_m_s=0.18,
            maximum_safe_speed_m_s=0.03,
            maximum_measured_speed_m_s=0.05,
            maximum_displacement_m=0.08,
        )

    def update(self, now_s, **overrides):
        values = {
            "eligible": True,
            "upstream_speed_m_s": 0.30,
            "safe_speed_m_s": 0.0,
            "measured_speed_m_s": 0.0,
            "position_x_m": 1.0,
            "position_y_m": 2.0,
        }
        values.update(overrides)
        return self.detector.update(now_s=now_s, **values)

    def test_transient_stop_does_not_trigger(self):
        self.assertFalse(self.update(10.0).ready)
        self.assertFalse(self.update(13.9).ready)
        self.assertTrue(self.update(14.0).ready)

    def test_zero_upstream_intent_resets_timer(self):
        self.update(10.0)
        decision = self.update(13.0, upstream_speed_m_s=0.0)
        self.assertFalse(decision.active)
        self.assertEqual(decision.reason, "no_motion_intent")
        self.assertFalse(self.update(14.1).ready)

    def test_safe_output_or_measured_motion_resets_timer(self):
        self.update(20.0)
        self.assertEqual(
            self.update(22.0, safe_speed_m_s=0.04).reason,
            "safe_command_nonzero",
        )
        self.update(23.0)
        self.assertEqual(
            self.update(25.0, measured_speed_m_s=0.06).reason,
            "robot_moving",
        )
        self.assertFalse(self.update(29.0).ready)

    def test_actual_displacement_resets_stall(self):
        self.update(30.0)
        decision = self.update(33.0, position_x_m=1.081)
        self.assertFalse(decision.active)
        self.assertEqual(decision.reason, "physical_progress")
        self.assertFalse(self.update(34.1, position_x_m=1.081).ready)

    def test_guard_dropout_resets_timer(self):
        self.update(40.0)
        self.assertEqual(self.update(43.0, eligible=False).reason, "guard_not_ready")
        self.assertFalse(self.update(44.1).ready)


class OmniEscapeGeometryTest(unittest.TestCase):
    @staticmethod
    def front_wall():
        return np.array(
            [(0.32, y_value) for y_value in np.linspace(-0.30, 0.30, 49)],
            dtype=np.float64,
        )

    def test_front_wall_rejects_forward_and_selects_safe_exit(self):
        decision = choose_escape_direction(self.front_wall(), 1.0, 0.0)
        self.assertTrue(decision.valid)
        self.assertLess(decision.direction_x, 0.95)
        self.assertTrue(
            direction_is_swept_clear(
                self.front_wall(),
                decision.direction_x,
                decision.direction_y,
                footprint_radius_m=0.265,
                travel_distance_m=0.30,
                clearance_margin_m=0.03,
                max_collision_points=3,
            )
        )

    def test_clear_path_direction_is_preferred_over_reverse(self):
        rear_wall = np.array(
            [(-0.32, y_value) for y_value in np.linspace(-0.30, 0.30, 49)],
            dtype=np.float64,
        )
        decision = choose_escape_direction(rear_wall, 1.0, 0.0)
        self.assertTrue(decision.valid)
        self.assertGreater(decision.direction_x, 0.95)
        self.assertAlmostEqual(decision.direction_y, 0.0, places=6)

    def test_swept_footprint_matches_stop_polygon_point_allowance(self):
        parameters = dict(
            footprint_radius_m=0.265,
            travel_distance_m=0.30,
            clearance_margin_m=0.03,
            max_collision_points=3,
        )
        self.assertFalse(
            direction_is_swept_clear(self.front_wall(), 1.0, 0.0, **parameters)
        )
        self.assertTrue(
            direction_is_swept_clear(self.front_wall(), -1.0, 0.0, **parameters)
        )
        four_contacts = [(0.35, offset) for offset in (-0.03, -0.01, 0.01, 0.03)]
        self.assertFalse(
            direction_is_swept_clear(four_contacts, 1.0, 0.0, **parameters)
        )

    def test_closed_ring_refuses_motion(self):
        angles = np.linspace(0.0, 2.0 * math.pi, 160, endpoint=False)
        points = np.column_stack((0.30 * np.cos(angles), 0.30 * np.sin(angles)))
        decision = choose_escape_direction(points, 1.0, 0.0)
        self.assertFalse(decision.valid)
        self.assertEqual(decision.reason, "no_safe_direction")
        self.assertEqual(decision.rejected_direction_count, 16)

    def test_sparse_returns_do_not_authorize_escape(self):
        decision = choose_escape_direction(
            [(0.30, 0.0)] * 7,
            1.0,
            0.0,
        )
        self.assertFalse(decision.valid)
        self.assertEqual(decision.reason, "insufficient_points")

    def test_supported_clearance_uses_evidence_threshold(self):
        points = [(0.30 + 0.01 * index, 0.0) for index in range(8)]
        self.assertAlmostEqual(
            supported_clearance(points, 0.0, 0.0, 0.265, 8),
            0.105,
        )
        self.assertIsNone(supported_clearance(points, 0.0, 0.0, 0.265, 9))

    def test_path_length_skips_non_finite_points(self):
        self.assertAlmostEqual(
            path_length(
                [(0.0, 0.0), (0.3, 0.4), (math.nan, 1.0), (0.6, 0.8)]
            ),
            1.0,
        )


if __name__ == "__main__":
    unittest.main()
