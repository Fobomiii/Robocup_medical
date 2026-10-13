"""Tests for the home-only continuous speed envelope."""

import math
import unittest

from obstacle_detector.home_approach_core import (
    approach_speed_limit,
    body_to_world_velocity,
    distance_to_home,
    home_approach_command,
    limit_bed_axis_velocity,
    limit_bed_body_velocity,
    limit_field_axis_velocity,
    limit_field_body_velocity,
    limit_home_axis_velocity,
    limit_home_body_velocity,
    limit_home_velocity,
    limit_velocity_toward_boundary,
    world_to_body_velocity,
)


class HomeApproachCoreTest(unittest.TestCase):
    def test_distance_uses_planar_home_geometry(self):
        self.assertAlmostEqual(distance_to_home(3.0, 4.0), 5.0)

    def test_envelope_is_full_speed_far_away(self):
        self.assertEqual(approach_speed_limit(10.0), 3.0)

    def test_envelope_decreases_continuously_toward_home(self):
        values = [approach_speed_limit(distance) for distance in (5.0, 4.0, 3.0, 1.0, 0.1)]
        self.assertEqual(values, sorted(values, reverse=True))
        self.assertAlmostEqual(values[-1], 0.10)
        self.assertGreater(values[1], values[2])

    def test_default_home_braking_envelope_begins_near_three_point_two_five_metres(self):
        self.assertAlmostEqual(approach_speed_limit(3.26), 3.0)
        self.assertLess(approach_speed_limit(3.25), 3.0)

    def test_explicit_home_envelope_begins_at_one_metre(self):
        options = {
            "max_speed_m_s": 3.0,
            "soft_decel_m_s2": 4.9875,
            "terminal_speed_m_s": 0.10,
            "terminal_distance_m": 0.10,
            "decel_start_distance_m": 1.00,
        }
        self.assertEqual(approach_speed_limit(1.01, **options), 3.0)
        self.assertEqual(approach_speed_limit(1.00, **options), 3.0)
        self.assertLess(approach_speed_limit(0.99, **options), 3.0)
        self.assertAlmostEqual(approach_speed_limit(0.10, **options), 0.10)

    def test_home_limit_caps_each_cross_drive_axis(self):
        vx, vy, limit = limit_home_velocity(3.0, 3.0, 0.0, 0.5)
        self.assertLessEqual(abs(vx), limit + 1.0e-9)
        self.assertLessEqual(abs(vy), limit + 1.0e-9)
        self.assertAlmostEqual(vx, vy)

    def test_bed_profile_caps_each_axis_near_bed_goal(self):
        vx, vy, limit = limit_home_velocity(
            1.5,
            1.5,
            5.4,
            1.2,
            home_x_m=5.4,
            home_y_m=2.2,
            max_speed_m_s=3.0,
            soft_decel_m_s2=1.0,
            terminal_speed_m_s=0.15,
            terminal_distance_m=0.20,
        )
        self.assertAlmostEqual(limit, math.sqrt(1.6225))
        self.assertLessEqual(abs(vx), limit + 1.0e-9)
        self.assertLessEqual(abs(vy), limit + 1.0e-9)
        self.assertAlmostEqual(vx, vy)

    def test_fast_bed_terminal_cap(self):
        options = {
            "max_speed_m_s": 3.0,
            "soft_decel_m_s2": 2.0,
            "terminal_speed_m_s": 0.45,
            "terminal_distance_m": 0.20,
        }
        self.assertAlmostEqual(approach_speed_limit(0.20, **options), 0.45)
        self.assertGreater(approach_speed_limit(0.50, **options), 0.45)

    def test_terminal_speed_inside_terminal_distance(self):
        self.assertAlmostEqual(approach_speed_limit(0.0), 0.10)

    def test_bed3_side_braking_preserves_forward_velocity(self):
        vx, vy, decision = limit_bed_axis_velocity(
            2.0,
            -1.5,
            2.0,
            -1.5,
            3.8,
            -2.0,
            target_x_m=5.4,
            target_y_m=-2.2,
            side_toward_sign=-1.0,
        )
        self.assertAlmostEqual(vx, 2.0)
        self.assertAlmostEqual(vy, 0.0)
        self.assertFalse(decision["x_limited"])
        self.assertTrue(decision["side_limited"])

    def test_bed_side_guard_is_symmetric(self):
        bed1_vx, bed1_vy, _ = limit_bed_axis_velocity(
            0.0, 0.8, 0.0, 0.4, 5.0, 2.0,
            target_x_m=5.4, target_y_m=2.2, side_toward_sign=1.0,
        )
        bed3_vx, bed3_vy, _ = limit_bed_axis_velocity(
            0.0, -0.8, 0.0, -0.4, 5.0, -2.0,
            target_x_m=5.4, target_y_m=-2.2, side_toward_sign=-1.0,
        )
        self.assertAlmostEqual(bed1_vx, bed3_vx)
        self.assertAlmostEqual(bed1_vy, -bed3_vy)

    def test_bed_boundary_blocks_only_further_motion(self):
        _, toward, _ = limit_bed_axis_velocity(
            0.0, -0.5, 0.0, -0.1, 5.4, -2.2,
            target_x_m=5.4, target_y_m=-2.2, side_toward_sign=-1.0,
        )
        _, away, _ = limit_bed_axis_velocity(
            0.0, 0.5, 0.0, -0.1, 5.4, -2.2,
            target_x_m=5.4, target_y_m=-2.2, side_toward_sign=-1.0,
        )
        self.assertEqual(toward, 0.0)
        self.assertEqual(away, 0.5)

    def test_bed_envelope_rotates_body_command_into_map_frame(self):
        forward, left, decision = limit_bed_body_velocity(
            1.0,
            0.0,
            0.0,
            -1.0,
            -math.pi / 2.0,
            5.4,
            -2.2,
            target_x_m=5.4,
            target_y_m=-2.2,
            side_toward_sign=-1.0,
        )
        self.assertAlmostEqual(forward, 0.0)
        self.assertAlmostEqual(left, 0.0)
        self.assertAlmostEqual(decision["input_world_vx_m_s"], 0.0)
        self.assertAlmostEqual(decision["input_world_vy_m_s"], -1.0)
        self.assertTrue(decision["side_limited"])

    def test_velocity_frame_rotations_are_inverse(self):
        world_vx, world_vy = body_to_world_velocity(1.2, -0.4, 0.37)
        forward, left = world_to_body_velocity(world_vx, world_vy, 0.37)
        self.assertAlmostEqual(forward, 1.2)
        self.assertAlmostEqual(left, -0.4)

    def test_field_left_and_right_edges_are_symmetric(self):
        left_vx, _, _ = limit_field_axis_velocity(
            -1.0, 0.0, -0.5, 0.0, -0.10, 0.0
        )
        right_vx, _, _ = limit_field_axis_velocity(
            1.0, 0.0, 0.5, 0.0, 6.10, 0.0
        )
        self.assertAlmostEqual(left_vx, -right_vx)

    def test_field_top_and_bottom_edges_are_symmetric(self):
        _, bottom_vy, _ = limit_field_axis_velocity(
            0.0, -1.0, 0.0, -0.5, 3.0, -3.10
        )
        _, top_vy, _ = limit_field_axis_velocity(
            0.0, 1.0, 0.0, 0.5, 3.0, 3.10
        )
        self.assertAlmostEqual(bottom_vy, -top_vy)

    def test_field_guard_preserves_parallel_motion(self):
        vx, vy, decision = limit_field_axis_velocity(
            1.0, 1.2, 0.8, 0.4, 6.10, 0.0
        )
        self.assertLess(vx, 1.0)
        self.assertAlmostEqual(vy, 1.2)
        self.assertTrue(decision["x_limited"])
        self.assertFalse(decision["y_limited"])

    def test_field_guard_allows_motion_back_inside_after_overshoot(self):
        vx, vy, decision = limit_field_axis_velocity(
            -0.7, 0.0, 0.2, 0.0, 6.30, 0.0
        )
        self.assertEqual((vx, vy), (-0.7, 0.0))
        self.assertFalse(decision["x_limited"])

    def test_field_corner_limits_both_axes_independently(self):
        vx, vy, decision = limit_field_axis_velocity(
            1.0, 1.0, 0.5, 0.5, 6.10, 3.10
        )
        self.assertLess(vx, 1.0)
        self.assertLess(vy, 1.0)
        self.assertTrue(decision["x_limited"])
        self.assertTrue(decision["y_limited"])

    def test_field_guard_rotates_body_command_at_45_degrees(self):
        forward, left, decision = limit_field_body_velocity(
            math.sqrt(2.0), 0.0, 0.5, 0.5, math.pi / 4.0, 6.10, 3.10
        )
        self.assertLess(math.hypot(forward, left), math.sqrt(2.0))
        self.assertTrue(decision["x_limited"])
        self.assertTrue(decision["y_limited"])

    def test_missing_wheels_do_not_stop_in_field_centre(self):
        vx, vy, decision = limit_field_axis_velocity(
            1.0, -0.5, None, None, 3.0, 0.0
        )
        self.assertEqual((vx, vy), (1.0, -0.5))
        self.assertTrue(decision["wheel_fallback"])

    def test_missing_wheels_still_brake_near_field_edge(self):
        vx, _, decision = limit_field_axis_velocity(
            1.0, 0.0, None, None, 6.10, 0.0
        )
        self.assertLess(vx, 1.0)
        self.assertTrue(decision["x_limited"])

    def test_field_usable_bounds_include_footprint_and_pose_margin(self):
        _, _, decision = limit_field_axis_velocity(
            0.0, 0.0, 0.0, 0.0, 3.0, 0.0
        )
        self.assertAlmostEqual(decision["usable_x_min_m"], -0.21)
        self.assertAlmostEqual(decision["usable_x_max_m"], 6.21)
        self.assertAlmostEqual(decision["usable_y_min_m"], -3.21)
        self.assertAlmostEqual(decision["usable_y_max_m"], 3.21)

    def test_field_guard_rejects_impossible_protected_rectangle(self):
        with self.assertRaises(ValueError):
            limit_field_axis_velocity(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                field_x_min_m=0.0, field_x_max_m=0.4,
                footprint_radius_m=0.23,
            )

    def test_stale_wheel_feedback_blocks_toward_bed(self):
        vx, vy, _ = limit_bed_axis_velocity(
            1.0, -1.0, None, None, 4.0, -1.0,
            target_x_m=5.4, target_y_m=-2.2, side_toward_sign=-1.0,
        )
        self.assertEqual((vx, vy), (0.0, 0.0))

    def test_home_boundary_limits_only_motion_through_start_line(self):
        vx, vy, decision = limit_home_axis_velocity(
            -1.5, 0.8, -1.0, 0.4, 0.35, 0.20,
        )
        self.assertGreater(vx, -1.5)
        self.assertAlmostEqual(vy, 0.8)
        self.assertTrue(decision["boundary_limited"])
        self.assertTrue(decision["braking"]["prebraking_active"])

    def test_dynamic_prebraking_matches_measured_stopping_distance(self):
        # 3.0 m/s with 0.22 s response, 2.0 m/s^2 braking and 0.03 m
        # margin requires 0.66 + 2.25 + 0.03 = 2.94 m.
        command, remaining, allowed, diagnostic = limit_velocity_toward_boundary(
            -3.0, -3.0, 2.94, 0.0, -1.0,
            braking_decel_m_s2=2.0,
            reaction_time_s=0.22,
            safety_margin_m=0.03,
            max_speed_m_s=3.0,
        )
        self.assertAlmostEqual(remaining, 2.94)
        self.assertAlmostEqual(diagnostic["required_stop_distance_m"], 2.94)
        self.assertAlmostEqual(allowed, 3.0)
        self.assertAlmostEqual(command, -3.0)

        command, _, allowed, diagnostic = limit_velocity_toward_boundary(
            -3.0, -3.0, 2.90, 0.0, -1.0,
            braking_decel_m_s2=2.0,
            reaction_time_s=0.22,
            safety_margin_m=0.03,
            max_speed_m_s=3.0,
        )
        self.assertGreater(command, -3.0)
        self.assertLess(allowed, 3.0)
        self.assertTrue(diagnostic["prebraking_active"])

    def test_bed_dynamic_prebraking_reports_each_axis(self):
        _, _, decision = limit_bed_axis_velocity(
            2.0, -1.0, 2.0, -1.0, 4.0, -1.5,
            target_x_m=5.4, target_y_m=-2.2, side_toward_sign=-1.0,
        )
        self.assertIn("required_stop_distance_m", decision["x_braking"])
        self.assertIn("required_stop_distance_m", decision["side_braking"])

    def test_home_boundary_keeps_escape_motion(self):
        vx, vy, decision = limit_home_axis_velocity(
            0.7, -0.6, 0.2, -0.1, -0.05, 0.10,
        )
        self.assertEqual((vx, vy), (0.7, -0.6))
        self.assertFalse(decision["boundary_limited"])

    def test_home_boundary_blocks_toward_motion_without_fresh_wheels(self):
        vx, vy, _ = limit_home_axis_velocity(
            -0.5, 0.4, None, None, 0.30, -0.10,
        )
        self.assertEqual(vx, 0.0)
        self.assertEqual(vy, 0.4)

    def test_home_envelope_rotates_body_command_into_map_frame(self):
        forward, left, decision = limit_home_body_velocity(
            1.0, 0.0, -1.0, 0.0, math.pi, 0.20, 0.0,
        )
        self.assertGreater(forward, 0.0)
        self.assertLess(forward, 1.0)
        self.assertAlmostEqual(left, 0.0)
        self.assertTrue(decision["boundary_limited"])

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
