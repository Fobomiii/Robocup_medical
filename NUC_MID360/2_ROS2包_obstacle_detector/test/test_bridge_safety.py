import unittest

from obstacle_detector.bridge_safety import (
    GateReleaseLimiter,
    SettledStopDetector,
    medical_mission_restarted,
    navigation_motion_is_authorized,
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
    def test_gate_release_limiter_uses_common_wheel_space_scale(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)

        output = limiter.update(1.0, 0.5, 0.4, 1.1)

        self.assertAlmostEqual(output[0], 0.125)
        self.assertAlmostEqual(output[1], 0.0625)
        self.assertAlmostEqual(output[2], 0.05)
        self.assertTrue(limiter.active)
        self.assertTrue(limiter.limiting)

    def test_gate_release_limiter_becomes_transparent_after_catchup(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)

        self.assertEqual(limiter.update(1.0, 0.0, 0.0, 1.5), (1.0, 0.0, 0.0))
        self.assertFalse(limiter.active)
        self.assertEqual(limiter.update(2.0, 0.0, 0.0, 1.51), (2.0, 0.0, 0.0))

    def test_gate_release_limiter_never_delays_stop_or_reduction(self):
        limiter = GateReleaseLimiter(2.0, 0.25, 0.25)
        limiter.reset(1.0)
        limiter.update(1.0, 0.5, 0.4, 1.1)

        self.assertEqual(limiter.update(0.05, 0.0, 0.0, 1.11), (0.05, 0.0, 0.0))
        self.assertTrue(limiter.active)
        self.assertEqual(limiter.update(0.0, 0.0, 0.0, 1.12), (0.0, 0.0, 0.0))

    def test_gate_release_limiter_ramps_after_safety_slowdown(self):
        limiter = GateReleaseLimiter(2.5, 0.25, 0.25)
        limiter.reset(1.0)
        output = limiter.update(1.0, 0.0, 0.0, 1.5)
        self.assertAlmostEqual(output[0], 1.0)
        self.assertEqual(output[1:], (0.0, 0.0))
        self.assertFalse(limiter.active)

        self.assertEqual(limiter.update(0.4, 0.0, 0.0, 1.6), (0.4, 0.0, 0.0))
        self.assertTrue(limiter.active)
        output = limiter.update(1.0, 0.0, 0.0, 1.8)
        self.assertAlmostEqual(output[0], 0.9)
        self.assertEqual(output[1:], (0.0, 0.0))
        self.assertTrue(limiter.limiting)

    def test_gate_release_limiter_ignores_gradual_bed_slowdown(self):
        limiter = GateReleaseLimiter(2.5, 0.25, 0.25)
        limiter.reset(1.0)
        self.assertEqual(limiter.update(1.0, 0.0, 0.0, 1.5), (1.0, 0.0, 0.0))
        self.assertFalse(limiter.active)

        self.assertEqual(limiter.update(0.85, 0.0, 0.0, 1.6), (0.85, 0.0, 0.0))
        self.assertEqual(limiter.update(0.70, 0.0, 0.0, 1.7), (0.70, 0.0, 0.0))
        self.assertFalse(limiter.active)
        self.assertEqual(limiter.update(1.0, 0.0, 0.0, 1.71), (1.0, 0.0, 0.0))

    def test_gate_release_limiter_rearms_after_authorization_is_revoked(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)
        limiter.update(1.0, 0.0, 0.0, 1.5)
        limiter.update(2.0, 0.0, 0.0, 1.51)

        limiter.reset(2.0)

        output = limiter.update(1.0, 0.0, 0.0, 2.1)
        self.assertAlmostEqual(output[0], 0.2)
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
