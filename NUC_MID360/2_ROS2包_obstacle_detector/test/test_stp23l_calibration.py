"""Protocol and guarded bedside OPS calibration regression tests."""

import os
import unittest

from obstacle_detector.nav_protocol import (
    FrameParser,
    MSG_STP23L,
    NAV_FOLLOWING,
    Stp23lTelemetry,
    decode_stp23l,
    encode_frame,
)
from obstacle_detector.stp23l_calibration import (
    OpsRangeCalibrator,
    load_calibration_config,
)


CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "config", "field_map.yaml"
)


class Stp23lCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.config = load_calibration_config(CONFIG)
        self.calibrator = OpsRangeCalibrator(self.config)

    def test_bed_references_match_tangent_circle_geometry(self):
        beds = {bed.name: bed for bed in self.config.beds}
        self.assertEqual(beds["bed1"].true_x_mm, -2200.0)
        self.assertEqual(beds["bed3"].true_x_mm, 2200.0)
        self.assertEqual(beds["bed1"].true_y_mm, 5400.0)
        self.assertEqual(beds["bed3"].true_y_mm, 5400.0)
        for bed in beds.values():
            self.assertEqual(bed.expected_center_side_distance_mm, 1300.0)
            self.assertEqual(bed.expected_center_front_distance_mm, 500.0)
            self.assertEqual(
                bed.expected_center_side_distance_mm
                - self.config.sensor_radius_mm,
                1147.0,
            )
            self.assertEqual(
                bed.expected_center_front_distance_mm
                - self.config.sensor_radius_mm,
                347.0,
            )

    def test_protocol_round_trip(self):
        payload = bytes.fromhex("04 7B 01 5B 04 7B 07")
        frame = FrameParser().feed(encode_frame(MSG_STP23L, 9, payload))[0]
        ranges = decode_stp23l(frame.payload)
        self.assertEqual(ranges, Stp23lTelemetry(1147, 347, 1147, 0x07))

    def test_bed1_corrects_surveyed_ops_to_true_circle_center(self):
        event = None
        # Approach the goal while the independently estimated offset remains
        # +50/+50 mm. Calibration should complete before Nav2 has to stop.
        for index in range(self.config.samples_required):
            raw_x = -2100.0 - index * 20.0
            raw_y = 5200.0 + index * 20.0
            actual_x = raw_x + 50.0
            actual_y = raw_y + 50.0
            ranges = Stp23lTelemetry(
                0,
                int(5900.0 - actual_y - 153.0),
                int(actual_x - (-3500.0) - 153.0),
                0x06,
            )
            event = self.calibrator.update(
                3, NAV_FOLLOWING, raw_x, raw_y, 0, ranges
            )
        self.assertTrue(event.applied)
        self.assertAlmostEqual(self.calibrator.offset_x_mm, 50.0)
        self.assertAlmostEqual(self.calibrator.offset_y_mm, 50.0)
        self.assertEqual(
            self.calibrator.corrected_xy(-2250.0, 5350.0), (-2200.0, 5400.0)
        )

    def test_bed3_uses_right_sensor_and_recalibrates_current_drift(self):
        ranges = Stp23lTelemetry(1147, 347, 0, 0x03)
        event = None
        for _ in range(self.config.samples_required):
            event = self.calibrator.update(
                6, NAV_FOLLOWING, 2150.0, 5450.0, 0, ranges
            )
        self.assertTrue(event.applied)
        self.assertAlmostEqual(self.calibrator.offset_x_mm, 50.0)
        self.assertAlmostEqual(self.calibrator.offset_y_mm, -50.0)

    def test_unexpected_obstacle_range_is_rejected(self):
        ranges = Stp23lTelemetry(0, 150, 400, 0x06)
        event = self.calibrator.update(
            3, NAV_FOLLOWING, -2250.0, 5350.0, 0, ranges
        )
        self.assertEqual(event.state, "range_outside_geometry_gate")
        self.assertFalse(event.applied)

    def test_leaving_bed_rearms_calibration_for_next_visit(self):
        ranges = Stp23lTelemetry(0, 347, 1147, 0x06)
        for _ in range(self.config.samples_required):
            first = self.calibrator.update(
                3, NAV_FOLLOWING, -2200.0, 5400.0, 0, ranges
            )
        self.assertTrue(first.applied)
        self.calibrator.update(9, NAV_FOLLOWING, 0.0, 0.0, 0, ranges)
        second = None
        for _ in range(self.config.samples_required):
            second = self.calibrator.update(
                3, NAV_FOLLOWING, -2200.0, 5400.0, 0, ranges
            )
        self.assertTrue(second.applied)


if __name__ == "__main__":
    unittest.main()
