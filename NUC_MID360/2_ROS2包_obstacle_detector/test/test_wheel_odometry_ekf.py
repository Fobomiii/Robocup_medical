import math
import struct
import unittest

from obstacle_detector.nav_protocol import decode_wheel_odom
from obstacle_detector.wheel_odometry_ekf import PlanarWheelOdometryEkf


class WheelOdomProtocolTest(unittest.TestCase):
    def test_decode(self):
        telemetry = decode_wheel_odom(
            struct.pack(">hhhBH", 1200, -350, 4500, 0x0F, 65000)
        )
        self.assertEqual(telemetry.forward_mm_s, 1200)
        self.assertEqual(telemetry.left_mm_s, -350)
        self.assertEqual(telemetry.yaw_ccw_cdeg_s, 4500)
        self.assertEqual(telemetry.online_mask, 0x0F)
        self.assertEqual(telemetry.stamp_cs, 65000)


class PlanarWheelOdometryEkfTest(unittest.TestCase):
    def test_forward_prediction_and_pose_correction(self):
        ekf = PlanarWheelOdometryEkf()
        ekf.update_wheel(1.0, 0.0, 0.0, 0x0F, 100)
        ekf.update_pose(0.0, 0.0, 0.0)
        ekf.update_wheel(1.0, 0.0, 0.0, 0x0F, 102)
        predicted_x = ekf.pose[0]
        self.assertAlmostEqual(predicted_x, 0.02, delta=0.005)

        ekf.update_pose(0.01, 0.0, 0.0)
        self.assertGreater(ekf.pose[0], 0.009)
        self.assertLess(ekf.pose[0], predicted_x)

    def test_body_velocity_rotates_into_world(self):
        ekf = PlanarWheelOdometryEkf()
        ekf.update_wheel(1.0, 0.0, 0.0, 0x0F, 10)
        ekf.update_pose(0.0, 0.0, math.pi / 2.0)
        ekf.update_wheel(1.0, 0.0, 0.0, 0x0F, 12)
        self.assertAlmostEqual(ekf.pose[0], 0.0, delta=0.005)
        self.assertAlmostEqual(ekf.pose[1], 0.02, delta=0.005)

    def test_invalid_motor_mask_is_rejected(self):
        ekf = PlanarWheelOdometryEkf()
        self.assertFalse(ekf.update_wheel(1.0, 0.0, 0.0, 0x07, 1))
        self.assertFalse(ekf.wheel_valid)

    def test_timestamp_wrap(self):
        ekf = PlanarWheelOdometryEkf()
        ekf.update_wheel(0.5, 0.0, 0.0, 0x0F, 65535)
        ekf.update_pose(0.0, 0.0, 0.0)
        ekf.update_wheel(0.5, 0.0, 0.0, 0x0F, 1)
        self.assertAlmostEqual(ekf.pose[0], 0.01, delta=0.004)


if __name__ == "__main__":
    unittest.main()
