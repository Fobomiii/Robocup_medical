"""Unit checks for the Mid360 chassis self-filter geometry."""

import unittest

import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from obstacle_detector.lidar_transform import (
    arm_filter_mask,
    forward_arm_extent,
    ground_filter_mask,
    ground_plane_removal_mask,
    height_ceiling_mask,
    quaternion_matrix,
    self_filter_mask,
    top_plate_filter_mask,
    transform_points,
    xyz_and_tag_from_cloud,
    xyz_from_cloud,
)


class LidarSelfFilterTest(unittest.TestCase):
    def test_urdf_mount_transform_matches_centered_upright_mid360(self):
        rotation = quaternion_matrix(0.0, 0.0, 0.0, 1.0)
        translation = np.array([0.0, 0.0, 0.33], dtype=np.float32)
        sensor_points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)

        actual = transform_points(sensor_points, rotation, translation)

        np.testing.assert_allclose(actual, [[1.0, 2.0, 3.33]], atol=1e-6)

    def test_quaternion_is_normalized_before_use(self):
        rotation = quaternion_matrix(0.0, 2.0, 0.0, 0.0)
        expected = np.diag([-1.0, 1.0, -1.0])
        np.testing.assert_allclose(rotation, expected, atol=1e-6)

    def test_filter_removes_only_points_inside_robot_cylinder(self):
        points = np.array(
            [
                [0.10, 0.10, 0.20],
                [0.26, 0.00, 0.20],
                [0.261, 0.00, 0.20],
                [0.10, 0.10, 0.50],
                [0.10, 0.10, -0.10],
            ],
            dtype=np.float32,
        )

        keep = self_filter_mask(points, radius=0.26, min_z=-0.05, max_z=0.45)

        np.testing.assert_array_equal(keep, [False, False, True, True, True])

    def test_output_height_ceiling_keeps_boundary_and_removes_higher_points(self):
        points = np.array(
            [[0.5, 0.0, 0.549], [0.5, 0.0, 0.550], [0.5, 0.0, 0.551]],
            dtype=np.float32,
        )

        keep = height_ceiling_mask(points, max_z=0.55)

        np.testing.assert_array_equal(keep, [True, True, False])

    def test_octagonal_top_plate_filters_only_its_high_slab(self):
        side = 0.18
        margin = 0.03
        apothem = side / (2.0 * np.tan(np.pi / 8.0)) + margin
        vertex_y = np.sqrt(2.0) * apothem - apothem
        points = np.array(
            [
                [apothem, 0.0, 0.60],
                [apothem, vertex_y, 0.52],
                [apothem + 0.001, 0.0, 0.60],
                [0.0, 0.0, 0.51],
                [0.0, 0.0, 0.66],
            ],
            dtype=np.float32,
        )

        keep = top_plate_filter_mask(
            points,
            center_x=0.0,
            center_y=0.0,
            side_length=side,
            margin=margin,
            min_z=0.52,
            max_z=0.65,
        )

        np.testing.assert_array_equal(keep, [False, False, True, True, True])

    ARM_BOX = dict(x_min=0.10, x_max=0.40, half_width=0.14, min_z=0.45, max_z=0.82)

    def test_arm_box_removes_the_forward_overhang_only(self):
        """The chassis cylinder cannot cover the arm, so a box must.

        The sweep box includes the measured arm plus motion and lidar jitter.
        It remains narrow enough that low obstacle returns still survive.
        """
        points = np.array(
            [
                # Arm returns: forward of the 0.26 m cylinder, inside the box.
                [0.355, 0.00, 0.62],
                [0.395, 0.13, 0.46],
                [0.10, -0.14, 0.45],
                # Low obstacle return below the arm sweep.
                [0.355, 0.00, 0.44],
                # A person beside the robot at arm height: outside the width.
                [0.355, 0.15, 0.62],
                # Behind the robot at the same height and width.
                [-0.355, 0.00, 0.62],
                # Just past the configured reach: must reappear, not vanish.
                [0.405, 0.00, 0.62],
            ],
            dtype=np.float32,
        )

        keep = arm_filter_mask(points, **self.ARM_BOX)

        np.testing.assert_array_equal(
            keep, [False, False, False, True, True, True, True]
        )

    def test_arm_sweep_box_preserves_low_obstacle_returns(self):
        """Nearby obstacles remain represented below the arm sweep."""
        heights = [0.08, 0.12, 0.30, 0.44, 0.45, 0.57, 0.60]
        probes = np.array(
            [[0.350, 0.0, z] for z in heights], dtype=np.float32
        )

        keep = arm_filter_mask(probes, **self.ARM_BOX)

        np.testing.assert_array_equal(keep, [True, True, True, True, False, False, False])

    def test_launched_arm_box_matches_the_costmap_and_the_urdf(self):
        """Read the values that actually ship instead of re-stating them.

        Three files describe the same box: the launch file the filter runs
        with, the costmap height band it must stay out of, and the URDF visual
        the operator compares against in RViz. A later edit to any one of them
        fails here rather than on the robot.
        """
        import re
        import xml.etree.ElementTree as ET
        from pathlib import Path

        root = Path(__file__).resolve().parents[1]
        launch = (root / "launch" / "obstacle.launch.py").read_text(encoding="utf-8")

        def launched(name):
            match = re.search(rf'"{name}":\s*([-\d.]+)', launch)
            self.assertIsNotNone(match, f"{name} missing from obstacle.launch.py")
            return float(match.group(1))

        output_max_z = launched("output_max_z")
        plate = {name: launched(f"top_plate_{name}") for name in
                 ("center_x", "center_y", "side_length", "margin", "min_z", "max_z")}
        box = {name: launched(f"arm_{name}") for name in
               ("x_min", "x_max", "half_width", "min_z", "max_z")}
        self.assertLess(box["x_min"], box["x_max"])
        self.assertLess(box["min_z"], box["max_z"])
        self.assertGreater(box["half_width"], 0.0)

        # The measured overhang must fit inside the clamp, otherwise the
        # sliver outside it flickers back into the navigation band.
        self.assertGreaterEqual(box["x_max"], 0.225 + 0.13)
        self.assertGreaterEqual(box["x_max"], 0.40)
        self.assertGreaterEqual(box["half_width"], 0.14)
        self.assertLessEqual(box["min_z"], 0.45)

        # Both costmaps and both collision monitors share this band.
        config = (root / "config" / "nav2_params.yaml").read_text(encoding="utf-8")
        ceilings = [float(value) for value in
                    re.findall(r"max_(?:obstacle_)?height:\s*([\d.]+)", config)]
        self.assertIn(0.60, ceilings)
        self.assertEqual(output_max_z, 0.55)
        self.assertLess(output_max_z, min(ceilings))
        self.assertEqual(plate["center_x"], 0.0)
        self.assertEqual(plate["center_y"], 0.0)
        self.assertEqual(plate["side_length"], 0.18)
        self.assertEqual(plate["margin"], 0.03)
        self.assertLessEqual(plate["min_z"], 0.55)
        self.assertGreaterEqual(plate["max_z"], 0.60)

        # The RViz visual is the same volume as the clamp.
        urdf = ET.parse(root / "config" / "robot.urdf").getroot()
        visual = urdf.find(".//link[@name='arm']/visual")
        size = [float(v) for v in visual.find("geometry/box").get("size").split()]
        origin = [float(v) for v in visual.find("origin").get("xyz").split()]
        self.assertAlmostEqual(box["x_min"], origin[0] - size[0] / 2.0, places=9)
        self.assertAlmostEqual(box["x_max"], origin[0] + size[0] / 2.0, places=9)
        self.assertAlmostEqual(box["half_width"], size[1] / 2.0, places=9)
        self.assertAlmostEqual(box["min_z"], origin[2] - size[2] / 2.0, places=9)
        self.assertAlmostEqual(box["max_z"], origin[2] + size[2] / 2.0, places=9)

    def test_forward_arm_extent_reports_reach_inside_the_width_band(self):
        points = np.array(
            [[0.20, 0.00, 0.40], [0.34, 0.10, 0.50], [0.90, 0.60, 0.40]],
            dtype=np.float32,
        )
        self.assertAlmostEqual(forward_arm_extent(points, 0.16), 0.34, places=6)
        self.assertEqual(forward_arm_extent(np.empty((0, 3), dtype=np.float32), 0.16), 0.0)

    def test_zero_length_quaternion_is_rejected(self):
        with self.assertRaises(ValueError):
            quaternion_matrix(0.0, 0.0, 0.0, 0.0)

    def test_ground_plane_is_removed_but_raised_obstacles_remain(self):
        x, y = np.meshgrid(
            np.linspace(-2.0, 2.0, 31), np.linspace(-2.0, 2.0, 31)
        )
        z = 0.025 * x - 0.015 * y
        floor = np.column_stack((x.ravel(), y.ravel(), z.ravel())).astype(np.float32)
        obstacle = np.array(
            [[1.0, 0.0, 0.12], [1.0, 0.0, 0.20], [-1.2, 0.5, 0.18]],
            dtype=np.float32,
        )
        points = np.vstack((floor, obstacle))

        keep, plane, ground_count = ground_filter_mask(
            points,
            distance_threshold=0.03,
            removal_below=0.04,
            removal_above=0.08,
            max_tilt_deg=12.0,
            max_origin_height=0.15,
            min_inliers=300,
            candidate_min_z=-0.25,
            candidate_max_z=0.30,
            min_radius=0.30,
            max_radius=5.0,
            iterations=40,
        )

        self.assertIsNotNone(plane)
        self.assertGreater(ground_count, 800)
        np.testing.assert_array_equal(keep[-3:], [True, True, True])

    def test_ground_filter_fails_open_without_enough_floor_points(self):
        points = np.array(
            [[0.5, 0.0, 0.2], [0.7, 0.2, 0.3], [1.0, -0.3, 0.4]],
            dtype=np.float32,
        )

        keep, plane, ground_count = ground_filter_mask(
            points, 0.04, 0.05, 0.08, 12.0, 0.15, 300,
            -0.25, 0.30, 0.30, 5.0, 40
        )

        np.testing.assert_array_equal(keep, [True, True, True])
        self.assertIsNone(plane)
        self.assertEqual(ground_count, 0)

    def test_cached_plane_removes_near_and_far_ground_without_hiding_obstacle(self):
        plane = np.array([0.02, -0.01, 0.99975, 0.0], dtype=np.float32)
        floor_xy = np.array(
            [[0.27, 0.0], [0.29, 0.0], [5.8, 0.0], [6.4, 0.0]],
            dtype=np.float32,
        )
        floor_z = -(floor_xy @ plane[:2]) / plane[2]
        floor = np.column_stack((floor_xy, floor_z)).astype(np.float32)
        obstacle = np.array([[1.0, 0.0, 0.14]], dtype=np.float32)
        points = np.vstack((floor, obstacle))

        keep, removed = ground_plane_removal_mask(
            points,
            plane,
            removal_below=0.06,
            removal_above=0.12,
            min_radius=0.26,
            max_radius=6.5,
        )

        np.testing.assert_array_equal(keep, [False, False, False, False, True])
        self.assertEqual(removed, 4)

    def test_mixed_type_livox_fields_are_supported(self):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="tag", offset=12, datatype=PointField.UINT8, count=1),
        ]
        cloud = pc2.create_cloud(Header(), fields, [(1.0, 2.0, 3.0, 7)])

        actual = xyz_from_cloud(cloud)

        np.testing.assert_allclose(actual, [[1.0, 2.0, 3.0]], atol=1e-6)

        _, tag = xyz_and_tag_from_cloud(cloud)
        np.testing.assert_array_equal(tag, [7])


if __name__ == "__main__":
    unittest.main()
