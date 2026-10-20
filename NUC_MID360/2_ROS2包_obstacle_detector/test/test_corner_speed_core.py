"""Tests for curvature-aware speed limits."""

import math
import unittest

from obstacle_detector.corner_speed_core import (
    CornerLimitReleaseLatch,
    CornerSpeedDecision,
    corner_speed_limit,
    reference_speed_profile,
)


def line_points(start_m: float, end_m: float, step_m: float = 0.10):
    point_count = int(round((end_m - start_m) / step_m))
    return [(start_m + point_index * step_m, 0.0) for point_index in range(point_count + 1)]


class CornerSpeedCoreTest(unittest.TestCase):
    def test_reference_profile_keeps_straight_path_at_route_speed(self):
        profile = reference_speed_profile(
            line_points(0.0, 6.0),
            0.0,
            0.0,
            3.0,
        )

        self.assertGreater(len(profile.samples), 50)
        self.assertTrue(
            all(sample.curvature_m_inv == 0.0 for sample in profile.samples)
        )
        self.assertTrue(
            all(sample.reference_speed_m_s == 3.0 for sample in profile.samples)
        )

    def test_reference_profile_uses_menger_curvature_on_a_circle(self):
        radius_m = 1.5
        path = [
            (
                radius_m * math.cos(math.radians(angle_deg)),
                radius_m * math.sin(math.radians(angle_deg)),
            )
            for angle_deg in range(0, 91, 2)
        ]
        profile = reference_speed_profile(
            path,
            path[0][0],
            path[0][1],
            3.0,
            curvature_span_m=0.30,
        )

        middle = profile.samples[len(profile.samples) // 2]
        self.assertAlmostEqual(middle.curvature_m_inv, 1.0 / radius_m, delta=0.03)
        self.assertAlmostEqual(middle.radius_m, radius_m, delta=0.08)

    def test_reference_profile_brakes_before_a_sharp_corner(self):
        path = line_points(0.0, 2.0)
        path.extend((2.0, 0.1 * value) for value in range(1, 31))

        profile = reference_speed_profile(path, 0.0, 0.0, 3.0)

        first = profile.samples[0]
        corner = max(profile.samples, key=lambda sample: sample.curvature_m_inv)
        self.assertEqual(first.curve_speed_m_s, 3.0)
        self.assertLess(first.braking_speed_m_s, first.curve_speed_m_s)
        self.assertLess(corner.curve_speed_m_s, 3.0)

    def test_reference_profile_respects_forward_acceleration_boundary(self):
        profile = reference_speed_profile(
            line_points(0.0, 3.0),
            0.0,
            0.0,
            0.0,
            accel_m_s2=4.0,
        )

        self.assertEqual(profile.samples[0].reference_speed_m_s, 0.0)
        for previous, current in zip(profile.samples, profile.samples[1:]):
            speed_squared_change = (
                current.reference_speed_m_s**2
                - previous.reference_speed_m_s**2
            )
            distance_change = current.distance_m - previous.distance_m
            self.assertLessEqual(
                speed_squared_change,
                2.0 * 4.0 * distance_change + 1.0e-9,
            )

    def test_reference_profile_rejects_negative_measured_speed(self):
        with self.assertRaises(ValueError):
            reference_speed_profile(
                line_points(0.0, 3.0),
                0.0,
                0.0,
                -0.1,
            )

    def test_straight_path_keeps_full_speed(self):
        decision = corner_speed_limit(line_points(0.0, 3.0), 0.0, 0.0)
        self.assertEqual(decision.speed_limit_m_s, 3.0)
        self.assertIsNone(decision.corner_distance_m)

    def test_ninety_degree_corner_is_limited_before_reaching_it(self):
        path = line_points(0.0, 1.0)
        path.extend((1.0, offset_m) for offset_m in [0.1 * value for value in range(1, 21)])
        decision = corner_speed_limit(path, 0.5, 0.0)
        self.assertLess(decision.speed_limit_m_s, 3.0)
        self.assertGreaterEqual(decision.speed_limit_m_s, 0.50)
        self.assertEqual(decision.corner_speed_m_s, 1.00)
        self.assertGreater(math.degrees(decision.turn_angle_rad), 30.0)

    def test_speed_limit_tightens_while_approaching_corner(self):
        path = line_points(0.0, 1.0)
        path.extend((1.0, offset_m) for offset_m in [0.1 * value for value in range(1, 21)])
        far_decision = corner_speed_limit(path, 0.0, 0.0)
        near_decision = corner_speed_limit(path, 0.8, 0.0)
        self.assertLess(near_decision.speed_limit_m_s, far_decision.speed_limit_m_s)
        self.assertGreaterEqual(near_decision.speed_limit_m_s, 0.50)

    def test_very_sharp_corner_can_be_slower_than_one_metre_per_second(self):
        path = line_points(0.0, 1.0, 0.05)
        path.extend(
            (1.0, 0.05 * value) for value in range(1, 21)
        )

        decision = corner_speed_limit(
            path,
            0.9,
            0.0,
            tangent_span_m=0.10,
            sample_step_m=0.05,
            braking_margin_m=0.20,
        )

        self.assertLess(decision.corner_speed_m_s, 1.00)
        self.assertGreaterEqual(decision.corner_speed_m_s, 0.50)

    def test_reaction_time_reserves_distance_before_corner(self):
        path = line_points(0.0, 2.4)
        path.extend(
            (2.4, 0.1 * value) for value in range(1, 21)
        )
        options = {
            "braking_decel_m_s2": 2.0,
            "lookahead_distance_m": 3.5,
            "tangent_span_m": 0.75,
            "min_turn_angle_rad": math.radians(35.0),
            "braking_margin_m": 0.20,
        }

        immediate = corner_speed_limit(
            path, 0.0, 0.0, reaction_time_s=0.0, **options
        )
        delayed = corner_speed_limit(
            path, 0.0, 0.0, reaction_time_s=0.20, **options
        )

        self.assertLess(delayed.speed_limit_m_s, immediate.speed_limit_m_s)
        self.assertGreaterEqual(delayed.speed_limit_m_s, 0.50)

    def test_negative_reaction_time_is_rejected(self):
        with self.assertRaises(ValueError):
            corner_speed_limit(
                line_points(0.0, 3.0),
                0.0,
                0.0,
                reaction_time_s=-0.01,
            )

    def test_corner_speed_ceiling_cannot_exceed_route_speed(self):
        with self.assertRaises(ValueError):
            corner_speed_limit(
                line_points(0.0, 3.0),
                0.0,
                0.0,
                max_speed_m_s=2.0,
                max_corner_speed_m_s=2.1,
            )

    def test_release_latch_requires_hold_and_straight_confirmation(self):
        latch = CornerLimitReleaseLatch(
            release_hold_s=0.50,
            straight_confirmation_s=0.20,
        )

        self.assertFalse(
            latch.update(
                10.0,
                corner_detected=True,
                straight_path_confirmed=False,
            )
        )
        self.assertTrue(
            latch.update(
                10.1,
                corner_detected=False,
                straight_path_confirmed=True,
            )
        )
        self.assertTrue(
            latch.update(
                10.49,
                corner_detected=False,
                straight_path_confirmed=True,
            )
        )
        self.assertFalse(
            latch.update(
                10.50,
                corner_detected=False,
                straight_path_confirmed=True,
            )
        )

    def test_release_latch_does_not_treat_missing_path_as_straight(self):
        latch = CornerLimitReleaseLatch(0.50, 0.20)
        latch.update(
            20.0,
            corner_detected=True,
            straight_path_confirmed=False,
        )

        self.assertTrue(
            latch.update(
                21.0,
                corner_detected=False,
                straight_path_confirmed=False,
            )
        )
        self.assertTrue(latch.armed)

    def test_three_metre_profile_sees_a_corner_beyond_old_lookahead(self):
        path = line_points(0.0, 2.8)
        path.extend(
            (2.8, 0.1 * value) for value in range(1, 21)
        )

        # Use a deliberately weaker braking envelope here so the test isolates
        # the 3.5 m lookahead instead of the deployed aggressive brake timing.
        decision = corner_speed_limit(
            path, 0.0, 0.0, braking_decel_m_s2=1.30
        )

        self.assertIsNotNone(decision.corner_distance_m)
        self.assertGreater(decision.corner_distance_m, 1.5)
        self.assertLess(decision.speed_limit_m_s, 3.0)

    def test_diagonal_to_horizontal_motion_is_detected_without_yaw(self):
        path = [(0.1 * value, 0.1 * value) for value in range(11)]
        path.extend((1.0 + 0.1 * value, 1.0) for value in range(1, 21))
        decision = corner_speed_limit(path, 0.7, 0.7)
        self.assertLess(decision.speed_limit_m_s, 3.0)
        self.assertGreaterEqual(decision.speed_limit_m_s, 0.50)
        self.assertGreater(math.degrees(decision.turn_angle_rad), 20.0)

    def test_projection_ignores_corners_already_behind_robot(self):
        path = line_points(0.0, 1.0)
        path.extend((1.0, offset_m) for offset_m in [0.1 * value for value in range(1, 11)])
        path.extend((1.0 + 0.1 * value, 1.0) for value in range(1, 21))
        decision = corner_speed_limit(path, 1.8, 1.0)
        self.assertEqual(decision.speed_limit_m_s, 3.0)

if __name__ == "__main__":
    unittest.main()
