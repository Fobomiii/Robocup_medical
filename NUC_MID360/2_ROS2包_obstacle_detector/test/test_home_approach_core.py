"""Tests for the home-only continuous speed envelope."""

import math
import unittest

from obstacle_detector.home_approach_core import (
    approach_speed_limit,
    distance_to_home,
    home_approach_command,
    limit_home_velocity,
)


class HomeApproachCoreTest(unittest.TestCase):
    def test_distance_uses_planar_home_geometry(self):
        self.assertAlmostEqual(distance_to_home(3.0, 4.0), 5.0)

    def test_envelope_is_full_speed_far_away(self):
        self.assertEqual(approach_speed_limit(10.0), 2.0)

    def test_envelope_decreases_continuously_toward_home(self):
        values = [approach_speed_limit(distance) for distance in (3.0, 2.0, 1.0, 0.5, 0.1)]
        self.assertEqual(values, sorted(values, reverse=True))
        self.assertAlmostEqual(values[-1], 0.10)
        self.assertGreater(values[1], values[2])

    def test_home_braking_envelope_begins_at_one_point_five_metres(self):
        self.assertAlmostEqual(approach_speed_limit(1.50), 2.0)
        self.assertLess(approach_speed_limit(1.49), 2.0)

    def test_scaling_preserves_direction_and_caps_speed(self):
        vx, vy, limit = limit_home_velocity(2.0, 2.0, 0.0, 0.5)
        self.assertLessEqual(math.hypot(vx, vy), limit + 1.0e-9)
        self.assertAlmostEqual(vx, vy)

    def test_bed_profile_caps_planar_speed_near_bed_goal(self):
        vx, vy, limit = limit_home_velocity(
            1.5,
            1.5,
            5.4,
            1.2,
            home_x_m=5.4,
            home_y_m=2.2,
            max_speed_m_s=2.0,
            soft_decel_m_s2=1.0,
            terminal_speed_m_s=0.15,
            terminal_distance_m=0.20,
        )
        self.assertAlmostEqual(limit, math.sqrt(1.6225))
        self.assertLessEqual(math.hypot(vx, vy), limit + 1.0e-9)
        self.assertAlmostEqual(vx, vy)

    def test_terminal_speed_inside_terminal_distance(self):
        self.assertAlmostEqual(approach_speed_limit(0.0), 0.10)

    def test_non_home_task_preserves_full_command(self):
        self.assertEqual(
            home_approach_command(
                1.8,
                1.2,
                home_active=False,
                pose_fresh=False,
            ),
            (1.8, 1.2),
        )

    def test_home_task_with_stale_pose_stops_before_smoother(self):
        self.assertEqual(
            home_approach_command(
                1.5,
                -0.5,
                home_active=True,
                pose_fresh=False,
            ),
            (0.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()
