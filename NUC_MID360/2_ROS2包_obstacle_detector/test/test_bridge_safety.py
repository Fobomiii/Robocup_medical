import unittest

from obstacle_detector.bridge_safety import (
    GateReleaseLimiter,
    SettledStopDetector,
    limit_planar_velocity,
    limit_xdrive_command,
    limit_yaw_rate_by_motion,
    medical_mission_restarted,
    navigation_motion_is_authorized,
    normalize_omni_command,
    omni_wheel_speeds,
)
from obstacle_detector.nav_protocol import (
    GOAL_BED1,
    GOAL_BED3,
    NAV_FOLLOWING,
    NAV_IDLE,
    NAV_WAIT_PATH,
    TASK_WAIT_START,
)


class BridgeSafetyTest(unittest.TestCase):
    def test_xdrive_uses_physical_45_degree_wheel_projection(self):
        axial = omni_wheel_speeds(2.0, 0.0, 0.0, 0.25)
        diagonal = omni_wheel_speeds(2.0 ** 0.5, 2.0 ** 0.5, 0.0, 0.25)

        self.assertAlmostEqual(max(abs(value) for value in axial), 2.0 ** 0.5)
        self.assertAlmostEqual(max(abs(value) for value in diagonal), 2.0)

    def test_wheel_normalization_leaves_single_axis_motion_unchanged(self):
        output = normalize_omni_command(2.0, 0.0, 0.0, 2.0, 0.25)
        self.assertEqual(output[:4], (2.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(
            max(
                abs(value)
                for value in omni_wheel_speeds(*output[:3], 0.25)
            ),
            2.0 ** 0.5,
        )

    def test_wheel_normalization_scales_diagonal_motion_together(self):
        vx, vy, wz, scale, requested_peak = normalize_omni_command(
            2.0, 1.5, 0.0, 2.0, 0.25
        )
        expected_peak = 3.5 / (2.0 ** 0.5)
        self.assertAlmostEqual(requested_peak, expected_peak)
        self.assertAlmostEqual(scale, 2.0 / expected_peak)
        self.assertAlmostEqual(vx / vy, 2.0 / 1.5)
        self.assertAlmostEqual(
            max(abs(value) for value in omni_wheel_speeds(vx, vy, wz, 0.25)),
            2.0,
        )

    def test_planar_limit_projects_square_command_to_two_m_per_s_circle(self):
        vx, vy, scale = limit_planar_velocity(2.0, 2.0, 2.0)

        self.assertAlmostEqual(vx, 2.0 ** 0.5)
        self.assertAlmostEqual(vy, 2.0 ** 0.5)
        self.assertAlmostEqual(scale, 1.0 / (2.0 ** 0.5))
        self.assertAlmostEqual((vx * vx + vy * vy) ** 0.5, 2.0)

    def test_yaw_limits_separate_heading_hold_from_active_rotation(self):
        heading_wz, heading_mode = limit_yaw_rate_by_motion(
            0.0, 1.0, 0.8, 0.2, 0.8, 0.05
        )
        rotation_wz, rotation_mode = limit_yaw_rate_by_motion(
            0.0, 0.0, 0.8, 0.2, 0.8, 0.05
        )

        self.assertEqual(heading_mode, "heading_correction")
        self.assertAlmostEqual(heading_wz, 0.2)
        self.assertEqual(rotation_mode, "active_rotation")
        self.assertAlmostEqual(rotation_wz, 0.8)

    def test_xdrive_command_reaches_two_m_per_s_in_diagonal_direction(self):
        output = limit_xdrive_command(
            2.0, 2.0, 0.0, 2.0, 2.0, 0.25, 0.2, 0.8, 0.05
        )
        vx, vy, wz, planar_scale, wheel_scale, requested_peak, yaw_mode = output

        self.assertAlmostEqual(vx, 2.0 ** 0.5)
        self.assertAlmostEqual(vy, 2.0 ** 0.5)
        self.assertEqual(wz, 0.0)
        self.assertAlmostEqual(planar_scale, 1.0 / (2.0 ** 0.5))
        self.assertEqual(wheel_scale, 1.0)
        self.assertAlmostEqual(requested_peak, 2.0)
        self.assertEqual(yaw_mode, "heading_correction")

    def test_heading_correction_keeps_yaw_and_reduces_translation_for_headroom(self):
        output = limit_xdrive_command(
            2.0 ** 0.5,
            2.0 ** 0.5,
            0.2,
            2.0,
            2.0,
            0.25,
            0.2,
            0.8,
            0.05,
        )
        vx, vy, wz, _, wheel_scale, _, yaw_mode = output

        self.assertAlmostEqual(wz, 0.2)
        self.assertAlmostEqual(wheel_scale, 0.975)
        self.assertAlmostEqual((vx * vx + vy * vy) ** 0.5, 1.95)
        self.assertEqual(yaw_mode, "heading_correction")
        self.assertAlmostEqual(
            max(abs(value) for value in omni_wheel_speeds(vx, vy, wz, 0.25)),
            2.0,
        )

    def test_wheel_normalization_includes_yaw_load(self):
        vx, vy, wz, scale, _ = normalize_omni_command(
            1.5, 0.5, 3.0, 2.0, 0.25
        )
        self.assertLess(scale, 1.0)
        self.assertAlmostEqual(
            max(abs(value) for value in omni_wheel_speeds(vx, vy, wz, 0.25)),
            2.0,
        )

    def test_gate_release_limiter_uses_common_wheel_space_scale(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)

        output = limiter.update(1.0, 0.5, 0.4, 1.1)

        self.assertAlmostEqual(output[0] / output[1], 2.0)
        self.assertAlmostEqual(output[0] / output[2], 2.5)
        self.assertAlmostEqual(
            max(abs(value) for value in limiter._wheel_speeds(output)), 0.2
        )
        self.assertTrue(limiter.active)
        self.assertTrue(limiter.limiting)

    def test_gate_release_limiter_stays_armed_after_catchup(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)

        self.assertEqual(limiter.update(1.0, 0.0, 0.0, 1.5), (1.0, 0.0, 0.0))
        self.assertTrue(limiter.active)
        output = limiter.update(2.0, 0.0, 0.0, 1.51)
        self.assertGreater(output[0], 1.0)
        self.assertLess(output[0], 2.0)
        self.assertTrue(limiter.limiting)

    def test_gate_release_limiter_never_delays_stop_or_reduction(self):
        limiter = GateReleaseLimiter(2.0, 0.25, 0.25)
        limiter.reset(1.0)
        limiter.update(1.0, 0.5, 0.4, 1.1)

        self.assertEqual(limiter.update(0.05, 0.0, 0.0, 1.11), (0.05, 0.0, 0.0))
        self.assertTrue(limiter.active)
        self.assertEqual(limiter.update(0.0, 0.0, 0.0, 1.12), (0.0, 0.0, 0.0))

    def test_gate_release_limiter_ramps_after_safety_slowdown(self):
        limiter = GateReleaseLimiter(2.0, 0.25, 0.25)
        limiter.reset(1.0)
        output = limiter.update(1.0, 0.0, 0.0, 1.5)
        self.assertAlmostEqual(output[0], 1.0)
        self.assertEqual(output[1:], (0.0, 0.0))
        self.assertTrue(limiter.active)

        self.assertEqual(limiter.update(0.4, 0.0, 0.0, 1.6), (0.4, 0.0, 0.0))
        self.assertTrue(limiter.active)
        output = limiter.update(1.0, 0.0, 0.0, 1.8)
        self.assertGreater(output[0], 0.4)
        self.assertLess(output[0], 1.0)
        self.assertEqual(output[1:], (0.0, 0.0))
        self.assertTrue(limiter.limiting)

    def test_gate_release_limiter_catches_recovery_after_gradual_slowdown(self):
        limiter = GateReleaseLimiter(2.5, 0.25, 0.25)
        limiter.reset(1.0)
        self.assertEqual(limiter.update(1.0, 0.0, 0.0, 1.5), (1.0, 0.0, 0.0))
        self.assertTrue(limiter.active)

        self.assertEqual(limiter.update(0.85, 0.0, 0.0, 1.6), (0.85, 0.0, 0.0))
        self.assertEqual(limiter.update(0.70, 0.0, 0.0, 1.7), (0.70, 0.0, 0.0))
        self.assertTrue(limiter.active)
        output = limiter.update(1.0, 0.0, 0.0, 1.71)
        self.assertGreater(output[0], 0.70)
        self.assertLess(output[0], 1.0)
        self.assertTrue(limiter.limiting)

    def test_gate_release_limiter_can_be_disabled_transparently(self):
        limiter = GateReleaseLimiter(0.0, 0.25)
        limiter.reset(1.0)

        self.assertEqual(
            limiter.update(1.5, -0.4, 0.3, 1.001),
            (1.5, -0.4, 0.3),
        )
        self.assertFalse(limiter.active)
        self.assertFalse(limiter.limiting)

    def test_gate_release_limiter_restarts_after_authorization_is_revoked(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)
        limiter.update(1.0, 0.0, 0.0, 1.5)
        limiter.update(2.0, 0.0, 0.0, 1.51)

        limiter.reset(2.0)

        output = limiter.update(1.0, 0.0, 0.0, 2.1)
        self.assertAlmostEqual(
            max(abs(value) for value in limiter._wheel_speeds(output)), 0.2
        )
        self.assertEqual(output[1:], (0.0, 0.0))
        self.assertTrue(limiter.limiting)

    def test_detects_stm32_restart_while_wait_state_is_unchanged(self):
        self.assertTrue(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                TASK_WAIT_START,
                NAV_WAIT_PATH,
                0.02,
                0.5,
            )
        )
        self.assertTrue(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                TASK_WAIT_START,
                NAV_FOLLOWING,
                0.8,
                0.5,
            )
        )
        self.assertFalse(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                TASK_WAIT_START,
                NAV_FOLLOWING,
                0.02,
                0.5,
            )
        )
        self.assertFalse(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                1,
                NAV_IDLE,
                1.0,
                0.5,
            )
        )

    def test_settled_stop_detector_waits_and_resets(self):
        detector = SettledStopDetector(0.03, 0.05, 0.15)

        self.assertFalse(detector.update(0.4, 0.0, 0.0, 1.0))
        self.assertFalse(detector.update(0.02, 0.0, 0.04, 1.1))
        self.assertFalse(detector.update(0.02, 0.0, 0.04, 1.24))
        self.assertTrue(detector.update(0.02, 0.0, 0.04, 1.251))
        self.assertFalse(detector.update(0.04, 0.0, 0.0, 1.3))
        self.assertFalse(detector.update(0.0, 0.0, 0.0, 1.4))

    def test_matching_active_goal_allows_navigation(self):
        self.assertTrue(
            navigation_motion_is_authorized(
                True, True, True, True, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )

    def test_old_goal_status_is_rejected_during_handoff(self):
        self.assertFalse(
            navigation_motion_is_authorized(
                True, True, True, True, 6, NAV_FOLLOWING, 13, GOAL_BED3,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )

    def test_waiting_path_and_stale_pose_are_rejected(self):
        self.assertFalse(
            navigation_motion_is_authorized(
                True, True, True, True, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_WAIT_PATH,
            )
        )
        self.assertFalse(
            navigation_motion_is_authorized(
                False, True, True, True, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )

    def test_handoff_hold_rejects_first_new_goal_commands(self):
        self.assertFalse(
            navigation_motion_is_authorized(
                True, True, False, True, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )

    def test_stale_navigator_status_revokes_motion(self):
        self.assertFalse(
            navigation_motion_is_authorized(
                True, False, True, True, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )

    def test_new_goal_following_hold_rejects_residual_command(self):
        self.assertFalse(
            navigation_motion_is_authorized(
                True, True, True, False, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )


if __name__ == "__main__":
    unittest.main()
