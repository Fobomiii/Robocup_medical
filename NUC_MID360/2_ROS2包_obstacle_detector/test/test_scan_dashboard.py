import unittest

from obstacle_detector.nav_protocol import (
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_BED3,
)
from obstacle_detector.scan_dashboard_core import parse_scan_result, parse_scan_status


class ScanDashboardCoreTest(unittest.TestCase):
    def test_parses_valid_result(self):
        self.assertEqual(
            parse_scan_result(
                '{"context": 2, "format": 2, "value": "6946522463487"}'
            ),
            (SCAN_CONTEXT_BED1, "6946522463487"),
        )

    def test_rejects_invalid_results(self):
        self.assertIsNone(parse_scan_result("not-json"))
        self.assertIsNone(parse_scan_result('{"context": 9, "value": "123"}'))
        self.assertIsNone(parse_scan_result('{"context": 3, "value": ""}'))

    def test_keeps_bed_contexts_independent(self):
        bed1 = parse_scan_result('{"context": 2, "value": "111"}')
        bed3 = parse_scan_result('{"context": 3, "value": "333"}')
        self.assertEqual(bed1, (SCAN_CONTEXT_BED1, "111"))
        self.assertEqual(bed3, (SCAN_CONTEXT_BED3, "333"))

    def test_parses_latched_scan_values(self):
        values = parse_scan_status(
            '{"camera": true, "scan_values": '
            '{"1": "31", "2": "6946522463487", "3": "6921361255288"}}'
        )
        self.assertEqual(values[SCAN_CONTEXT_BED1], "6946522463487")
        self.assertEqual(values[SCAN_CONTEXT_BED3], "6921361255288")

    def test_rejects_status_without_scan_values(self):
        self.assertIsNone(parse_scan_status('{"camera": true}'))


if __name__ == "__main__":
    unittest.main()
