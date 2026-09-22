"""Unit checks for the Mid360 chassis self-filter geometry."""

import unittest

import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from obstacle_detector.lidar_transform import (
    ground_filter_mask,
    quaternion_matrix,
    self_filter_mask,
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
