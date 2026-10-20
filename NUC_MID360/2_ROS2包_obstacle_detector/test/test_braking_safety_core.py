"""Tests for the observation-only physical braking model."""

import math
import unittest

from obstacle_detector.braking_safety_core import (
    braking_speed_limit,
    evaluate_braking_safety,
    stopping_distance,
)


def repeated_point(x_m, y_m, count=5):
    return [(x_m, y_m) for _ in range(count)]


class BrakingSafetyCoreTest(unittest.TestCase):
    def test_no_obstacle_keeps_command_speed(self):
        decision = evaluate_braking_safety([], 2.0, 0.0, 1.8, 0.0)

        self.assertEqual(decision.speed_limit_m_s, 3.0)
        self.assertEqual(decision.shadow_speed_m_s, 2.0)
        self.assertFalse(decision.would_limit)

    def test_forward_obstacle_applies_continuous_limit(self):
        decision = evaluate_braking_safety(
            repeated_point(1.0, 0.0), 3.0, 0.0, 3.0, 0.0
        )

        self.assertAlmostEqual(decision.collision_distance_m, 0.755, places=6)
        self.assertGreater(decision.speed_limit_m_s, 0.0)
        self.assertLess(decision.speed_limit_m_s, 3.0)
        self.assertTrue(decision.would_limit)

    def test_obstacle_at_margin_commands_complete_stop(self):
        decision = evaluate_braking_safety(
            repeated_point(0.30, 0.0),
            1.0,
            0.0,
            1.0,
            0.0,
            safety_margin_m=0.15,
        )

        self.assertLess(decision.collision_distance_m, 0.15)
        self.assertEqual(decision.effective_distance_m, 0.0)
        self.assertEqual(decision.shadow_speed_m_s, 0.0)
        self.assertTrue(decision.would_stop)

    def test_side_and_rear_points_do_not_limit(self):
        points = repeated_point(-1.0, 0.0) + repeated_point(1.0, 0.30)
        decision = evaluate_braking_safety(points, 2.0, 0.0, 2.0, 0.0)

        self.assertIsNone(decision.collision_distance_m)
        self.assertFalse(decision.would_limit)

    def test_diagonal_motion_uses_diagonal_sweep(self):
        diagonal = 1.0 / math.sqrt(2.0)
        decision = evaluate_braking_safety(
            repeated_point(diagonal, diagonal),
            2.0,
            2.0,
            2.0,
            2.0,
        )

        self.assertAlmostEqual(decision.collision_distance_m, 0.755, places=6)
        self.assertTrue(decision.would_limit)

    def test_minimum_point_threshold_rejects_sparse_return(self):
        decision = evaluate_braking_safety(
            repeated_point(0.8, 0.0, count=4), 2.0, 0.0, 2.0, 0.0
        )

        self.assertEqual(decision.candidate_point_count, 4)
        self.assertIsNone(decision.collision_distance_m)
        self.assertFalse(decision.would_limit)

    def test_invalid_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            braking_speed_limit(1.0, 0.0, 0.2, 3.0)
        with self.assertRaises(ValueError):
            stopping_distance(1.0, 2.5, -0.1)
        with self.assertRaises(ValueError):
            evaluate_braking_safety([], 1.0, 0.0, 1.0, 0.0, min_points=0)
        with self.assertRaises(ValueError):
            evaluate_braking_safety([], 0.0, 0.0, 0.0, 0.0, max_speed_m_s=0.0)

    def test_speed_limit_increases_monotonically_with_distance(self):
        limits = [braking_speed_limit(distance, 2.5, 0.2, 3.0)
                  for distance in (0.0, 0.5, 1.0, 2.0, 4.0)]

        self.assertEqual(limits, sorted(limits))
        self.assertEqual(limits[0], 0.0)
        self.assertEqual(limits[-1], 3.0)

    def test_stopping_distance_includes_reaction_and_braking(self):
        self.assertAlmostEqual(stopping_distance(3.0, 2.5, 0.2), 2.4)


if __name__ == "__main__":
    unittest.main()
