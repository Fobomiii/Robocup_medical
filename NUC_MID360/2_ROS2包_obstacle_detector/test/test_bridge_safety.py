import unittest

from obstacle_detector.bridge_safety import (
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
