import math
import os
import unittest

from obstacle_detector.nurse_scan_core import (
    field_yaw_toward,
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

    def test_all_viewpoints_face_qr(self):
        for viewpoint in self.config.viewpoints:
            yaw_deg = field_yaw_toward(
                viewpoint.x_mm,
                viewpoint.y_mm,
                self.config.qr_x_mm,
                self.config.qr_y_mm,
            )
            yaw_rad = math.radians(yaw_deg)
            heading_x = math.sin(yaw_rad)
            heading_y = math.cos(yaw_rad)
            qr_x = self.config.qr_x_mm - viewpoint.x_mm
            qr_y = self.config.qr_y_mm - viewpoint.y_mm
            cross = heading_x * qr_y - heading_y * qr_x
            self.assertAlmostEqual(cross, 0.0, places=6)
            self.assertGreater(heading_x * qr_x + heading_y * qr_y, 0.0)

    def test_left_and_right_yaw_are_mirrored(self):
        viewpoints = {
            viewpoint.name: viewpoint for viewpoint in self.config.viewpoints
        }
        center = viewpoints["nurse_scan_center"]
        left = viewpoints["nurse_scan_left"]
        right = viewpoints["nurse_scan_right"]
        left_yaw = field_yaw_toward(
            left.x_mm, left.y_mm, self.config.qr_x_mm, self.config.qr_y_mm
        )
        center_yaw = field_yaw_toward(
            center.x_mm, center.y_mm, self.config.qr_x_mm, self.config.qr_y_mm
        )
        right_yaw = field_yaw_toward(
            right.x_mm, right.y_mm, self.config.qr_x_mm, self.config.qr_y_mm
        )
        self.assertGreater(left_yaw, 0.0)
        self.assertAlmostEqual(center_yaw, 0.0)
        self.assertAlmostEqual(left_yaw, -right_yaw)

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
