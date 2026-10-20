import math
import unittest

from obstacle_detector.mcu_time import (
    align_pose_measurements,
    elapsed_u32_ms,
    sample_ros_time_ns,
)


class McuTimeTest(unittest.TestCase):
    def test_tick_wraparound(self):
        self.assertEqual(elapsed_u32_ms(5, 0xFFFFFFF0), 21)

    def test_sample_time_uses_transmit_age(self):
        self.assertEqual(
            sample_ros_time_ns(1000, 1040, 10_000_000_000),
            9_960_000_000,
        )

    def test_invalid_or_stale_stamp_falls_back(self):
        self.assertIsNone(sample_ros_time_ns(0, 1040, 10_000_000_000))
        self.assertIsNone(sample_ros_time_ns(1, 2000, 10_000_000_000))

    def test_asynchronous_pose_is_propagated_to_newer_sample(self):
        aligned = align_pose_measurements(
            1.0, 2.0, 0.0,
            1_000_000_000, 1_100_000_000,
            2.0, 0.0, 0.0,
        )
        self.assertAlmostEqual(aligned.x, 1.2)
        self.assertAlmostEqual(aligned.y, 2.0)
        self.assertEqual(aligned.stamp_ns, 1_100_000_000)
        self.assertAlmostEqual(aligned.yaw, 0.0)


if __name__ == "__main__":
    unittest.main()
