"""GM65 framing, hotplug, bedside gating and shared camera-result tests."""

import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from obstacle_detector.gm65_reader import DEFAULT_GM65_DEVICE, Gm65Reader
from obstacle_detector.nav_protocol import (
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_BED3,
    SCAN_CONTEXT_ORDER,
    SCAN_FORMAT_CODE128,
    SCAN_FORMAT_QR,
)
from obstacle_detector.scanner_core import ScanConsensus


BARCODE = "6946522463487"
OTHER_BARCODE = "6921361255288"
PACKAGE = Path(__file__).resolve().parents[1]


class FakeSerial:
    def __init__(self):
        self.buffer = bytearray()
        self.closed = False
        self.failure = ""

    @property
    def in_waiting(self):
        if self.failure == "waiting":
            raise OSError("USB disconnected")
        return len(self.buffer)

    def read(self, size):
        if self.failure == "read":
            raise OSError("USB read failed")
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def reset_input_buffer(self):
        if self.failure == "reset":
            raise OSError("USB reset failed")
        self.buffer.clear()

    def close(self):
        self.closed = True


class Gm65ReaderTests(unittest.TestCase):
    def setUp(self):
        self.serial = FakeSerial()
        self.factory = Mock(return_value=self.serial)
        self.reader = Gm65Reader(serial_factory=self.factory)
        self.reader.poll(SCAN_CONTEXT_BED1, 1.0)

    def test_stable_device_nonblocking_and_exclusive_open(self):
        self.factory.assert_called_once_with(
            port=DEFAULT_GM65_DEVICE,
            baudrate=9600,
            timeout=0,
            write_timeout=0,
            exclusive=True,
        )

    def test_one_complete_frame_with_split_serial_reads(self):
        self.serial.buffer.extend(BARCODE[:5].encode())
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.02), [])
        self.serial.buffer.extend((BARCODE[5:] + "\r\n").encode())
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.04), [BARCODE])
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.06), [])

    def test_no_suffix_requires_idle_boundary(self):
        self.serial.buffer.extend(BARCODE.encode())
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.02), [])
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.10), [])
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.13), [BARCODE])

    def test_delayed_fragment_with_new_bytes_is_not_mistaken_for_idle(self):
        self.serial.buffer.extend(BARCODE[:5].encode())
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.02), [])
        self.serial.buffer.extend(BARCODE[5:].encode() + b"\r")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.2), [BARCODE])

    def test_delayed_extra_bytes_do_not_accept_a_whitelist_prefix(self):
        self.serial.buffer.extend(BARCODE.encode())
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.02), [])
        self.serial.buffer.extend(b"9\r")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.2), [])

    def test_qr_unknown_prefix_and_non_ascii_are_rejected(self):
        for data in (
            b"11\r\n", b"31\n", b"1234567890123\n",
            b"prefix" + BARCODE.encode() + b"\n",
            b"\xff" + BARCODE.encode() + b"\n",
            BARCODE.encode() + OTHER_BARCODE.encode() + b"\n",
        ):
            with self.subTest(data=data):
                self.serial.buffer.extend(data)
                self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.02), [])

    def test_multiple_frames_preserve_order(self):
        self.serial.buffer.extend((BARCODE + "\r" + OTHER_BARCODE + "\n").encode())
        self.assertEqual(
            self.reader.poll(SCAN_CONTEXT_BED1, 1.02), [BARCODE, OTHER_BARCODE]
        )

    def test_oversized_frame_is_discarded_until_boundary(self):
        self.serial.buffer.extend(b"x" * 100 + BARCODE.encode() + b"\r")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.02), [])
        self.serial.buffer.extend(BARCODE.encode() + b"\r")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.04), [BARCODE])

    def test_closed_gate_and_entering_bed_discard_queued_data(self):
        for context in (None, SCAN_CONTEXT_ORDER, 99):
            with self.subTest(context=context):
                self.serial.buffer.extend(BARCODE.encode() + b"\n")
                self.assertEqual(self.reader.poll(context, 1.02), [])
                self.assertFalse(self.serial.buffer)
        self.serial.buffer.extend(BARCODE.encode() + b"\n")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.04), [])
        self.assertFalse(self.serial.buffer)
        self.serial.buffer.extend(BARCODE.encode() + b"\n")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.06), [BARCODE])

    def test_partial_frame_cannot_cross_beds(self):
        self.serial.buffer.extend(BARCODE[:6].encode())
        self.reader.poll(SCAN_CONTEXT_BED1, 1.02)
        self.serial.buffer.extend(BARCODE[6:].encode() + b"\n")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED3, 1.04), [])
        self.serial.buffer.extend(BARCODE[6:].encode() + b"\n")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED3, 1.06), [])

    def test_reset_drops_partial_and_queued_frames_in_same_bed(self):
        self.serial.buffer.extend(BARCODE[:6].encode())
        self.reader.poll(SCAN_CONTEXT_BED1, 1.02)
        self.reader.reset()
        self.serial.buffer.extend(BARCODE.encode() + b"\n")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.04), [])
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.20), [])

    def test_delayed_poll_drops_old_input(self):
        self.serial.buffer.extend(BARCODE.encode() + b"\n")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.6), [])
        self.assertFalse(self.serial.buffer)

    def test_large_backlog_is_flushed_instead_of_replayed(self):
        self.serial.buffer.extend((BARCODE.encode() + b"\n") * 100)
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.02), [])
        self.assertFalse(self.serial.buffer)

    def test_missing_device_retries_without_sleep(self):
        factory = Mock(side_effect=[OSError("No such device"), FakeSerial()])
        reader = Gm65Reader(serial_factory=factory)
        self.assertEqual(reader.poll(SCAN_CONTEXT_BED1, 1.0), [])
        self.assertFalse(reader.connected)
        self.assertEqual(reader.last_error, "No such device")
        self.assertEqual(reader.poll(SCAN_CONTEXT_BED1, 2.0), [])
        self.assertEqual(factory.call_count, 1)
        self.assertEqual(reader.poll(SCAN_CONTEXT_BED1, 3.0), [])
        self.assertTrue(reader.connected)
        self.assertEqual(reader.last_error, "")

    def test_disconnect_and_reconnect_drop_stale_frames(self):
        self.serial.failure = "waiting"
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 1.02), [])
        self.assertFalse(self.reader.connected)
        self.assertTrue(self.serial.closed)
        replacement = FakeSerial()
        replacement.buffer.extend(BARCODE.encode() + b"\n")
        self.factory.return_value = replacement
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 3.03), [])
        self.assertTrue(self.reader.connected)
        self.assertFalse(replacement.buffer)
        replacement.buffer.extend(BARCODE.encode() + b"\n")
        self.assertEqual(self.reader.poll(SCAN_CONTEXT_BED1, 3.05), [BARCODE])

    def test_read_and_reset_failures_are_contained(self):
        for operation in ("read", "reset"):
            with self.subTest(operation=operation):
                connection = FakeSerial()
                reader = Gm65Reader(serial_factory=Mock(return_value=connection))
                reader.poll(SCAN_CONTEXT_BED1, 1.0)
                connection.failure = operation
                connection.buffer.extend(BARCODE.encode() + b"\n")
                context = None if operation == "reset" else SCAN_CONTEXT_BED1
                self.assertEqual(reader.poll(context, 1.02), [])
                self.assertFalse(reader.connected)
                self.assertTrue(connection.closed)


def load_scanner_without_ros():
    names = (
        "cv2", "rclpy", "rclpy.node", "zxingcpp", "ament_index_python",
        "ament_index_python.packages", "geometry_msgs", "geometry_msgs.msg",
        "pyzbar", "pyzbar.pyzbar", "sensor_msgs", "sensor_msgs.msg",
        "std_msgs", "std_msgs.msg",
    )
    modules = {name: ModuleType(name) for name in names}
    modules["rclpy.node"].Node = object
    modules["ament_index_python.packages"].get_package_share_directory = Mock()
    modules["geometry_msgs.msg"].PoseStamped = SimpleNamespace
    modules["pyzbar.pyzbar"].ZBarSymbol = SimpleNamespace
    modules["pyzbar.pyzbar"].decode = Mock()
    modules["sensor_msgs.msg"].CompressedImage = SimpleNamespace
    modules["std_msgs.msg"].String = SimpleNamespace
    modules["std_msgs.msg"].UInt8 = SimpleNamespace
    spec = importlib.util.spec_from_file_location(
        "obstacle_detector._gm65_scanner_test",
        PACKAGE / "obstacle_detector" / "code_scanner.py",
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class Gm65IntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_scanner_without_ros()

    def setUp(self):
        self.clock = patch.object(self.module.time, "monotonic", return_value=10.0)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = self.module.CodeScanner.__new__(self.module.CodeScanner)
        self.node.task_state = 3
        self.node.last_task_state_s = 10.0
        self.node.pose_timeout_s = 0.5
        self.node.last_pose_s = 10.0
        self.node.robot_field_pose_mm = (-2200.0, 5400.0)
        self.node.scan_targets_mm = {
            SCAN_CONTEXT_BED1: (-2200.0, 5400.0),
            SCAN_CONTEXT_BED3: (2200.0, 5400.0),
        }
        self.node.bed_scan_activation_distance_mm = 1500.0
        self.node.scan_values = dict.fromkeys(
            (SCAN_CONTEXT_ORDER, SCAN_CONTEXT_BED1, SCAN_CONTEXT_BED3), ""
        )
        self.node.tele_state_lock = threading.RLock()
        self.node.tele_scan_succeeded = False
        self.node.result_pub = Mock()
        self.node.get_logger = Mock(return_value=Mock())
        self.node.consensus = ScanConsensus(2, 0.8)
        self.node.gm65 = Mock()
        self.node.gm65_last_error = ""
        self.node.mission_epoch = 1
        self.node._publish_status = Mock()

    def test_only_current_bed_navigation_or_scan_states_are_allowed(self):
        for task_state in range(16):
            self.node.task_state = task_state
            self.node.robot_field_pose_mm = (
                (2200.0, 5400.0) if task_state in (6, 7) else (-2200.0, 5400.0)
            )
            expected = {3: SCAN_CONTEXT_BED1, 4: SCAN_CONTEXT_BED1,
                        6: SCAN_CONTEXT_BED3, 7: SCAN_CONTEXT_BED3}.get(task_state)
            with self.subTest(task_state=task_state):
                self.assertEqual(self.node._gm65_context(), expected)

    def test_far_wrong_bed_missing_or_stale_pose_is_rejected(self):
        for pose in (None, (0.0, 0.0), (2200.0, 5400.0), (-2200.0, 3899.0)):
            self.node.robot_field_pose_mm = pose
            self.assertIsNone(self.node._gm65_context())
        self.node.robot_field_pose_mm = (-2200.0, 5400.0)
        self.node.last_pose_s = 9.0
        self.assertIsNone(self.node._gm65_context())
        self.node.last_pose_s = 10.0
        self.node.last_task_state_s = 9.0
        self.assertIsNone(self.node._gm65_context())

    def test_one_gm65_frame_uses_existing_result_protocol(self):
        self.node.gm65.connected = True
        self.node.gm65.last_error = ""
        self.node.gm65.poll.return_value = [BARCODE, OTHER_BARCODE]
        self.node._gm65_tick()
        self.node.result_pub.publish.assert_called_once()
        result = json.loads(self.node.result_pub.publish.call_args.args[0].data)
        self.assertEqual(result, {
            "context": SCAN_CONTEXT_BED1, "format": SCAN_FORMAT_CODE128,
            "value": BARCODE, "source": "gm65",
        })
        self.assertIsNone(self.node._gm65_context())
        self.assertFalse(self.node.tele_scan_succeeded)

    def test_first_camera_or_gm65_result_wins(self):
        for first, second in (("gm65", "zbar"), ("zbar", "gm65")):
            with self.subTest(first=first):
                self.node.scan_values[SCAN_CONTEXT_BED1] = ""
                self.node.result_pub.reset_mock()
                self.assertTrue(self.node._confirm_scan(
                    SCAN_CONTEXT_BED1, SCAN_FORMAT_CODE128, BARCODE, first
                ))
                self.assertFalse(self.node._confirm_scan(
                    SCAN_CONTEXT_BED1, SCAN_FORMAT_CODE128, OTHER_BARCODE, second
                ))
                self.node.result_pub.publish.assert_called_once()
                self.assertEqual(self.node.last_source, first)

    def test_invalid_result_or_wrong_context_is_not_published(self):
        for context, scan_format, value in (
            (SCAN_CONTEXT_ORDER, SCAN_FORMAT_QR, "11"),
            (SCAN_CONTEXT_BED3, SCAN_FORMAT_CODE128, BARCODE),
            (SCAN_CONTEXT_BED1, SCAN_FORMAT_CODE128, "1234567890123"),
        ):
            self.assertFalse(self.node._confirm_scan(context, scan_format, value, "gm65"))
        self.node.result_pub.publish.assert_not_called()

    def test_tele_nurse_qr_still_confirms_normally(self):
        self.node.task_state = 14
        self.assertFalse(self.node._confirm_scan(
            SCAN_CONTEXT_ORDER, SCAN_FORMAT_QR, "31", "gm65"
        ))
        self.assertTrue(self.node._confirm_scan(
            SCAN_CONTEXT_ORDER, SCAN_FORMAT_QR, "31", "tele_zbar"
        ))
        self.assertTrue(self.node.tele_scan_succeeded)
        self.assertIsNone(self.node._gm65_context())

    def test_serial_failure_leaves_camera_results_available(self):
        self.node.gm65.connected = False
        self.node.gm65.last_error = "USB disconnected"
        self.node.gm65.poll.return_value = []
        self.node._gm65_tick()
        self.assertTrue(self.node._confirm_scan(
            SCAN_CONTEXT_BED1, SCAN_FORMAT_CODE128, BARCODE, "zbar"
        ))
        self.node.result_pub.publish.assert_called_once()

    def test_tick_discards_unexpected_results_when_gate_is_closed(self):
        self.node.task_state = 14
        self.node.gm65.connected = True
        self.node.gm65.last_error = ""
        self.node.gm65.poll.return_value = [BARCODE]
        self.node._gm65_tick()
        self.node.gm65.poll.assert_called_once_with(None, 10.0)
        self.node.result_pub.publish.assert_not_called()

    def test_task_change_resets_reader_but_keeps_cached_bed_scan(self):
        self.node.scan_values[SCAN_CONTEXT_BED1] = BARCODE
        self.node._task_state(SimpleNamespace(data=12))
        self.node.gm65.reset.assert_called_once()
        self.assertEqual(self.node.scan_values[SCAN_CONTEXT_BED1], BARCODE)
        self.assertIsNone(self.node._gm65_context())

    def test_mission_reset_clears_reader_and_cached_scan(self):
        self.node.scan_values[SCAN_CONTEXT_BED1] = BARCODE
        self.node._bridge_status(SimpleNamespace(data=json.dumps({
            "mission_epoch": 2, "stm32": {"task_state": 14},
        })))
        self.node.gm65.reset.assert_called_once()
        self.assertFalse(any(self.node.scan_values.values()))

    def test_config_keeps_camera_confirmation_and_existing_position_gate(self):
        config = yaml.safe_load((PACKAGE / "config" / "scanner.yaml").read_text(
            encoding="utf-8"
        ))["code_scanner"]["ros__parameters"]
        self.assertTrue(config["gm65_enabled"])
        self.assertEqual(config["gm65_device"], DEFAULT_GM65_DEVICE)
        self.assertEqual(config["gm65_baud"], 9600)
        self.assertEqual(config["confirm_hits"], 2)
        self.assertEqual(config["bed_scan_activation_distance_m"], 1.5)


if __name__ == "__main__":
    unittest.main()
