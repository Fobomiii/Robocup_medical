import unittest

from obstacle_detector.controller_selection import (
    FOLLOW_PATH_HEADING_HOLD,
    FOLLOW_PATH_LATERAL_HOLD,
    FOLLOW_PATH_NURSE,
    controller_for_route,
)


class ControllerSelectionTest(unittest.TestCase):
    def test_nurse_approach_and_scan_viewpoints_allow_mppi_rotation(self):
        for goal_name in (
            "nurse",
            "nurse_scan_entry",
            "nurse_scan_center",
            "nurse_scan_left",
            "nurse_scan_right",
        ):
            self.assertEqual(
                controller_for_route("home", goal_name), FOLLOW_PATH_NURSE
            )

    def test_bed_to_bed_uses_lateral_exploration_in_both_directions(self):
        self.assertEqual(
            controller_for_route("bed1", "bed3"), FOLLOW_PATH_LATERAL_HOLD
        )
        self.assertEqual(
            controller_for_route("bed3", "bed1"), FOLLOW_PATH_LATERAL_HOLD
        )

    def test_both_beds_return_home_with_heading_hold_profile(self):
        self.assertEqual(
            controller_for_route("bed1", "home"), FOLLOW_PATH_HEADING_HOLD
        )
        self.assertEqual(
            controller_for_route("bed3", "home"), FOLLOW_PATH_HEADING_HOLD
        )

    def test_non_bed_to_bed_routes_default_to_heading_hold(self):
        self.assertEqual(
            controller_for_route("nurse", "bed1"), FOLLOW_PATH_HEADING_HOLD
        )
        self.assertEqual(
            controller_for_route("nurse", "bed3"), FOLLOW_PATH_HEADING_HOLD
        )

    def test_unknown_goal_fails_closed(self):
        with self.assertRaises(ValueError):
            controller_for_route("bed1", "unconfigured_goal")


if __name__ == "__main__":
    unittest.main()
