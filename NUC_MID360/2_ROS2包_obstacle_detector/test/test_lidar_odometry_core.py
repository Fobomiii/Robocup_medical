"""Unit tests for the OPS-versus-Livox motion comparison core."""

import math
import unittest

import numpy as np

from obstacle_detector.lidar_odometry_core import (
    Pose2D,
    dilate_mask,
    interpolate_pose,
    points_in_grid_mask,
    relative_motion,
    transform_points,
    translation_icp,
    voxel_downsample,
)


class LidarOdometryCoreTest(unittest.TestCase):
    def test_relative_motion_is_expressed_in_previous_body_frame(self):
        previous = Pose2D(0, 1.0, 2.0, math.pi / 2.0)
        current = Pose2D(1, 1.0, 2.4, math.pi / 2.0 + 0.1)

        translation, yaw = relative_motion(previous, current)

        np.testing.assert_allclose(translation, [0.4, 0.0], atol=1e-9)
        self.assertAlmostEqual(yaw, 0.1)

    def test_pose_interpolation_handles_wrapped_yaw(self):
        samples = [
            Pose2D(0, 0.0, 0.0, math.radians(179.0)),
            Pose2D(1_000_000_000, 1.0, 2.0, math.radians(-179.0)),
        ]

        pose = interpolate_pose(samples, 500_000_000, 0.2)

        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.x, 0.5)
        self.assertAlmostEqual(pose.y, 1.0)
        self.assertAlmostEqual(abs(pose.yaw), math.pi, places=6)

    def test_dilated_grid_mask_is_bounded_and_fast_to_query(self):
        occupied = np.zeros((5, 5), dtype=bool)
        occupied[2, 2] = True
        support = dilate_mask(occupied, 1)
        points = np.array(
            [[2.5, 2.5], [1.5, 2.5], [0.5, 0.5], [5.5, 2.5]]
        )

        keep = points_in_grid_mask(points, support, 0.0, 0.0, 1.0)

        np.testing.assert_array_equal(keep, [True, True, False, False])

    def test_translation_icp_recovers_motion_with_outliers(self):
        horizontal = np.column_stack(
            (np.linspace(-1.5, 1.5, 80), np.zeros(80))
        )
        vertical = np.column_stack(
            (np.zeros(60), np.linspace(0.0, 1.8, 60))
        )
        previous = voxel_downsample(np.vstack((horizontal, vertical)), 0.04)
        expected_translation = np.array([0.18, -0.07])
        yaw = 0.06
        # previous = R(yaw) * current + translation
        current = transform_points(previous - expected_translation, np.zeros(2), -yaw)
        current = np.vstack(
            (current, [[2.5, 2.5], [2.7, -2.0], [-2.2, 2.4]])
        )

        result = translation_icp(
            current,
            previous,
            yaw,
            initial_translation=np.array([0.14, -0.03]),
            max_correspondence_distance=0.20,
            max_iterations=10,
        )

        self.assertIsNotNone(result)
        np.testing.assert_allclose(
            result.translation, expected_translation, atol=0.025
        )
        self.assertGreater(result.inlier_ratio, 0.90)
        self.assertLess(result.rmse, 0.05)
        self.assertGreater(result.spatial_extent, 2.0)

    def test_single_line_is_reported_as_degenerate(self):
        line = np.column_stack((np.linspace(-2.0, 2.0, 100), np.zeros(100)))
        corner = np.vstack((line, np.column_stack((np.zeros(100), np.linspace(0, 2, 100)))))

        line_result = translation_icp(
            line - np.array([0.10, 0.0]), line, 0.0, np.array([0.08, 0.0]), 0.2, 8
        )
        corner_result = translation_icp(
            corner - np.array([0.10, 0.0]),
            corner,
            0.0,
            np.array([0.08, 0.0]),
            0.2,
            8,
        )

        self.assertIsNotNone(line_result)
        self.assertIsNotNone(corner_result)
        self.assertLess(line_result.geometry_ratio, 0.02)
        self.assertGreater(corner_result.geometry_ratio, 0.05)

    def test_person_sized_cluster_has_insufficient_spatial_extent(self):
        angles = np.linspace(0.0, 2.0 * math.pi, 100, endpoint=False)
        previous = np.column_stack((0.25 * np.cos(angles), 0.20 * np.sin(angles)))
        current = previous - np.array([0.08, -0.03])

        result = translation_icp(
            current, previous, 0.0, np.array([0.06, -0.02]), 0.2, 8
        )

        self.assertIsNotNone(result)
        self.assertLess(result.spatial_extent, 1.0)


if __name__ == "__main__":
    unittest.main()
