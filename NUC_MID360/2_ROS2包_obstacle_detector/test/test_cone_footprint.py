"""Unit checks for competition-cone ground-footprint compensation."""

import unittest

import numpy as np

from obstacle_detector.cone_footprint_core import expand_cone_footprints


class ConeFootprintTest(unittest.TestCase):
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
        radii = np.hypot(
            synthetic[:, 0] - center_x,
            synthetic[:, 1] - center_y,
        )
        self.assertAlmostEqual(float(radii.max()), 0.155, places=3)
        np.testing.assert_allclose(synthetic[:, 2], 0.12, atol=1e-6)

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
