import struct
import unittest

from obstacle_detector.nav_protocol import decode_pose


class PoseTimeProtocolTest(unittest.TestCase):
    def test_legacy_pose_payload_remains_supported(self):
        pose = decode_pose(struct.pack(">iihBBHB", 1, 2, 3, 4, 5, 6, 7))
        self.assertEqual((pose.x_mm, pose.y_mm, pose.yaw_cdeg), (1, 2, 3))
        self.assertEqual(pose.ops_sample_ms, 0)
        self.assertEqual(pose.hwt_yaw_sample_ms, 0)
        self.assertEqual(pose.tx_stamp_ms, 0)

    def test_extended_pose_payload_carries_both_sensor_times(self):
        pose = decode_pose(
            struct.pack(">iihBBHBIII", 1, 2, 3, 4, 5, 6, 7, 101, 102, 110)
        )
        self.assertEqual(pose.ops_sample_ms, 101)
        self.assertEqual(pose.hwt_yaw_sample_ms, 102)
        self.assertEqual(pose.tx_stamp_ms, 110)


if __name__ == "__main__":
    unittest.main()
