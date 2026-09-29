import unittest

from obstacle_detector.nav_protocol import (
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_BED3,
)
from obstacle_detector.scan_dashboard_core import (
    START_ERROR,
    START_READY,
    START_RUNNING,
    START_SUCCESS,
    START_WAITING,
    describe_start_status,
    parse_bridge_status,
    parse_scan_result,
    parse_scan_status,
    parse_tele_scan_state,
)


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

    def test_parses_tele_scan_failure(self):
        self.assertEqual(
            parse_tele_scan_state('{"tele_scan_state": "failed"}'),
            "failed",
        )
        self.assertIsNone(parse_tele_scan_state('{"tele_scan_state": "bad"}'))

    def test_parses_start_gate_after_path_is_ready(self):
        status = parse_bridge_status(
            '{"serial": true, "pose": true, "motion_authorized": false, '
            '"start_gate": {"waiting_for_button": true, '
            '"path_ready": true, "can_start": true}, '
            '"stm32": {"task_state": 14, "nav_status": 2}}'
        )
        self.assertTrue(status["waiting_for_button"])
        self.assertTrue(status["path_ready"])
        self.assertTrue(status["can_start"])
        self.assertFalse(status["motion_authorized"])

    def test_start_status_covers_wait_ready_run_and_success(self):
        def status(task_state, can_start=False):
            return parse_bridge_status(
                '{"serial": true, "pose": true, "stm32": {"task_state": %d, '
                '"nav_status": 2}, "start_gate": {"waiting_for_button": true, '
                '"path_ready": %s, "can_start": %s}}'
                % (task_state, str(can_start).lower(), str(can_start).lower())
            )

        waiting = describe_start_status(status(14))
        self.assertEqual(waiting[0], START_WAITING)
        self.assertEqual(waiting[1], "等待")
        self.assertEqual(waiting[2], "#e53935")

        ready = describe_start_status(status(15, can_start=True))
        self.assertEqual(ready[0], START_READY)
        self.assertEqual(ready[1], "发车")
        self.assertEqual(ready[2], "#138a36")

        for driving in (1, 3, 6, 12, 13):
            running = describe_start_status(status(driving, can_start=True))
            self.assertEqual(running[0], START_RUNNING, driving)
            self.assertEqual(running[1], "运行")
            self.assertEqual(running[2], "#138a36")

        home = describe_start_status(status(9, can_start=True))
        self.assertEqual(home[0], START_RUNNING)

        success = describe_start_status(status(10))
        self.assertEqual(success[0], START_SUCCESS)
        self.assertEqual(success[1], "成功")
        self.assertEqual(success[2], "#138a36")

        failure = describe_start_status(status(11))
        self.assertEqual(failure[0], START_ERROR)
        self.assertEqual(failure[1], "失败")
        self.assertEqual(failure[2], "#e53935")

    def test_start_status_falls_back_to_waiting(self):
        self.assertEqual(describe_start_status(None)[0], START_WAITING)
        unknown = parse_bridge_status('{"stm32": {"task_state": 99}}')
        self.assertEqual(describe_start_status(unknown)[0], START_WAITING)


if __name__ == "__main__":
    unittest.main()
