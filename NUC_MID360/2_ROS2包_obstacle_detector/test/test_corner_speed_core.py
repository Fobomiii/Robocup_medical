"""Tests for curvature-aware speed limits."""

import math
import unittest

from obstacle_detector.corner_speed_core import corner_speed_limit


def line_points(start_m: float, end_m: float, step_m: float = 0.10):
    point_count = int(round((end_m - start_m) / step_m))
    return [(start_m + point_index * step_m, 0.0) for point_index in range(point_count + 1)]


class CornerSpeedCoreTest(unittest.TestCase):
    def test_straight_path_keeps_full_speed(self):
        decision = corner_speed_limit(line_points(0.0, 3.0), 0.0, 0.0)
        self.assertEqual(decision.speed_limit_m_s, 2.0)
        self.assertIsNone(decision.corner_distance_m)

    def test_ninety_degree_corner_is_limited_before_reaching_it(self):
        path = line_points(0.0, 1.0)
        path.extend((1.0, offset_m) for offset_m in [0.1 * value for value in range(1, 21)])
        decision = corner_speed_limit(path, 0.5, 0.0)
        self.assertLess(decision.speed_limit_m_s, 2.0)
        self.assertGreaterEqual(decision.speed_limit_m_s, 1.0)
        self.assertGreater(math.degrees(decision.turn_angle_rad), 30.0)

    def test_speed_limit_tightens_while_approaching_corner(self):
        path = line_points(0.0, 1.0)
        path.extend((1.0, offset_m) for offset_m in [0.1 * value for value in range(1, 21)])
        far_decision = corner_speed_limit(path, 0.0, 0.0)
        near_decision = corner_speed_limit(path, 0.8, 0.0)
        self.assertLess(near_decision.speed_limit_m_s, far_decision.speed_limit_m_s)
        self.assertGreaterEqual(near_decision.speed_limit_m_s, 1.0)

    def test_diagonal_to_horizontal_motion_is_detected_without_yaw(self):
        path = [(0.1 * value, 0.1 * value) for value in range(11)]
        path.extend((1.0 + 0.1 * value, 1.0) for value in range(1, 21))
        decision = corner_speed_limit(path, 0.7, 0.7)
        self.assertLess(decision.speed_limit_m_s, 2.0)
        self.assertGreaterEqual(decision.speed_limit_m_s, 1.0)
        self.assertGreater(math.degrees(decision.turn_angle_rad), 20.0)

    def test_projection_ignores_corners_already_behind_robot(self):
        path = line_points(0.0, 1.0)
        path.extend((1.0, offset_m) for offset_m in [0.1 * value for value in range(1, 11)])
        path.extend((1.0 + 0.1 * value, 1.0) for value in range(1, 21))
        decision = corner_speed_limit(path, 1.8, 1.0)
        self.assertEqual(decision.speed_limit_m_s, 2.0)


if __name__ == "__main__":
    unittest.main()
