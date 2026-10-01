"""Unit checks for competition-cone ground-footprint compensation."""

import unittest

import numpy as np

from obstacle_detector.cone_footprint_core import expand_cone_footprints


class ConeFootprintTest(unittest.TestCase):
    def test_compensator_keeps_tf_reception_parallel_with_cloud_processing(self):
        """Exact-time persistence lookups must not starve TF callbacks."""
        from pathlib import Path

        source = (
            Path(__file__).resolve().parents[1]
            / "obstacle_detector"
            / "cone_footprint.py"
        ).read_text(encoding="utf-8")

        self.assertIn("from rclpy.executors import MultiThreadedExecutor", source)
        self.assertIn("MultiThreadedExecutor(num_threads=2)", source)
        self.assertIn("executor.spin()", source)

    def test_compact_vertical_return_adds_known_base_disk(self):
        cone = np.array(
            [
                [1.00, -0.05, 0.18],
                [1.01, 0.05, 0.20],
                [1.00, -0.03, 0.34],
                [1.02, 0.03, 0.48],
            ],
            dtype=np.float32,
        )

        expanded, centers = expand_cone_footprints(cone)

        self.assertEqual(len(centers), 1)
        self.assertGreater(len(expanded), len(cone) + 20)
        synthetic = expanded[len(cone):]
        center_x, center_y = centers[0]
        self.assertGreater(center_x, float(np.median(cone[:, 0])) + 0.04)
        radii = np.hypot(
            synthetic[:, 0] - center_x,
            synthetic[:, 1] - center_y,
        )
        self.assertAlmostEqual(float(radii.max()), 0.18, places=3)
        np.testing.assert_allclose(synthetic[:, 2], 0.12, atol=1e-6)

    def test_visible_surface_is_shifted_back_to_known_cone_axis(self):
        axis_x = 1.20
        physical_radius = 0.155
        cone_height = 0.65
        heights = np.array([0.15, 0.25, 0.35, 0.45], dtype=np.float32)
        surface_radii = physical_radius * (1.0 - heights / cone_height)
        cone = np.column_stack(
            (axis_x - surface_radii, np.zeros_like(heights), heights)
        ).astype(np.float32)

        expanded, centers = expand_cone_footprints(
            cone,
            base_radius=0.165,
            physical_base_radius=physical_radius,
            cone_height=cone_height,
        )

        self.assertEqual(len(centers), 1)
        self.assertAlmostEqual(centers[0][0], axis_x, places=3)
        self.assertAlmostEqual(centers[0][1], 0.0, places=3)
        synthetic = expanded[len(cone):]
        self.assertAlmostEqual(float(np.min(synthetic[:, 0])), 1.035, places=3)

    def test_wide_fixture_is_not_expanded(self):
        fixture = np.array(
            [
                [1.0, -0.30, 0.10],
                [1.0, -0.15, 0.25],
                [1.0, 0.00, 0.40],
                [1.0, 0.15, 0.55],
                [1.0, 0.30, 0.65],
            ],
            dtype=np.float32,
        )

        expanded, centers = expand_cone_footprints(fixture)

        self.assertEqual(centers, [])
        np.testing.assert_array_equal(expanded, fixture)

    def test_high_cluster_never_reaching_the_ground_is_not_expanded(self):
        """The mounted arm fits the cone test but must not get a base disk.

        Measured arm underside at 0.64-0.70 m, 0.11 m past the body face, so
        it lands inside the 0.08-0.68 m candidate band and looks compact. A
        disk for it would be a false wall at z=0.12 m right at the robot's
        front edge -- the exact stall this guard prevents.
        """
        arm = np.array(
            [
                [0.32, -0.10, 0.64],
                [0.33, 0.00, 0.66],
                [0.32, 0.10, 0.65],
                [0.335, 0.00, 0.70],
            ],
            dtype=np.float32,
        )

        expanded, centers = expand_cone_footprints(arm)

        self.assertEqual(centers, [])
        np.testing.assert_array_equal(expanded, arm)

    def test_flat_noise_is_not_expanded(self):
        noise = np.array(
            [[1.0, offset, 0.20] for offset in (-0.04, 0.0, 0.04)],
            dtype=np.float32,
        )

        expanded, centers = expand_cone_footprints(noise)

        self.assertEqual(centers, [])
        np.testing.assert_array_equal(expanded, noise)


if __name__ == "__main__":
    unittest.main()
