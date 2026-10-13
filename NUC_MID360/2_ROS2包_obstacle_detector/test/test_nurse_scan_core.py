import os
import unittest

from obstacle_detector.nurse_scan_core import (
    load_nurse_scan_config,
    next_nurse_viewpoint_index,
)


class NurseScanCoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_nurse_scan_config(
            os.path.join(os.path.dirname(__file__), "..", "config", "field_map.yaml")
        )

    def test_zone_contains_original_nurse_goal(self):
        self.assertTrue(self.config.zone.contains(0.0, 1700.0))
        self.assertFalse(self.config.zone.contains(0.0, 1000.0))

    def test_viewpoint_order_is_center_left_right(self):
        self.assertEqual(
            [viewpoint.name for viewpoint in self.config.viewpoints],
            ["nurse_scan_center", "nurse_scan_left", "nurse_scan_right"],
        )

    def test_viewpoint_selection_alternates_sides_after_center(self):
        next_index = 0
        selected = []
        for _ in range(7):
            selected_index = next_nurse_viewpoint_index(
                next_index, len(self.config.viewpoints)
            )
            selected.append(self.config.viewpoints[selected_index].name)
            next_index = selected_index + 1

        self.assertEqual(
            selected,
            [
                "nurse_scan_center",
                "nurse_scan_left",
                "nurse_scan_right",
                "nurse_scan_left",
                "nurse_scan_right",
                "nurse_scan_left",
                "nurse_scan_right",
            ],
        )


if __name__ == "__main__":
    unittest.main()
