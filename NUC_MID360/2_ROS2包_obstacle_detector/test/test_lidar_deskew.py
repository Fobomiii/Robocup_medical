import math
import unittest

import numpy as np

from obstacle_detector.lidar_deskew import (
    MotionSample,
    deskew_points,
    interpolate_motion,
)


class LidarDeskewTest(unittest.TestCase):
    def test_translation_deskew_removes_scan_stretch(self):
        samples = [
            MotionSample(0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            MotionSample(100_000_000, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0),
        ]
        points = np.asarray([[2.0, 0.0, 0.2], [1.9, 0.0, 0.2]])
        result = deskew_points(
            points,
            np.asarray([0, 100_000_000]),
            samples,
            100_000_000,
            20_000_000,
        )
        np.testing.assert_allclose(result[:, 0], [1.9, 1.9], atol=1.0e-6)

    def test_yaw_deskew_aligns_fixed_world_point(self):
        quarter_turn = math.pi / 2.0
        samples = [
            MotionSample(0, 0.0, 0.0, 0.0, 0.0, 0.0, quarter_turn),
            MotionSample(
                1_000_000_000, 0.0, 0.0, quarter_turn,
                0.0, 0.0, quarter_turn,
            ),
        ]
        points = np.asarray([[1.0, 0.0, 0.2], [0.0, -1.0, 0.2]])
        result = deskew_points(
            points,
            np.asarray([0, 1_000_000_000]),
            samples,
            1_000_000_000,
            20_000_000,
        )
        np.testing.assert_allclose(result[:, :2], [[0.0, -1.0], [0.0, -1.0]], atol=1.0e-6)

    def test_missing_history_rejects_compensation(self):
        sample = MotionSample(1_000_000_000, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
        result = interpolate_motion(
            [sample], np.asarray([800_000_000]), 50_000_000
        )
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
