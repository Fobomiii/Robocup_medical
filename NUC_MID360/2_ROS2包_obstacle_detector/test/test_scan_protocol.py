"""Camera scan protocol, whitelist and temporal-confirmation tests."""

import unittest

from obstacle_detector.nav_protocol import (
    FrameParser,
    MSG_SCAN_ACK,
    MSG_SCAN_RESULT,
    SCAN_ACK_ACCEPTED,
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_ORDER,
    SCAN_FORMAT_CODE128,
    SCAN_FORMAT_QR,
    ScanAck,
    ScanResult,
    decode_scan_ack,
    decode_scan_result,
    encode_frame,
    encode_scan_ack,
    encode_scan_result,
)
from obstacle_detector.scanner_core import (
    ScanConsensus,
    scan_matches_task,
)


class ScanProtocolTest(unittest.TestCase):
    def test_scan_result_frame_round_trip(self):
        payload = encode_scan_result(
            513, SCAN_CONTEXT_BED1, SCAN_FORMAT_CODE128, "6946522463487"
        )
        frame = FrameParser().feed(encode_frame(MSG_SCAN_RESULT, 7, payload))[0]
        self.assertEqual(frame.msg_type, MSG_SCAN_RESULT)
        self.assertEqual(
            decode_scan_result(frame.payload),
            ScanResult(513, SCAN_CONTEXT_BED1, SCAN_FORMAT_CODE128, "6946522463487"),
        )

    def test_scan_ack_round_trip(self):
        payload = encode_scan_ack(65535, SCAN_ACK_ACCEPTED)
        frame = FrameParser().feed(encode_frame(MSG_SCAN_ACK, 8, payload))[0]
        self.assertEqual(decode_scan_ack(frame.payload), ScanAck(65535, SCAN_ACK_ACCEPTED))

    def test_scan_result_rejects_bad_lengths_and_non_ascii(self):
        with self.assertRaises(ValueError):
            encode_scan_result(1, SCAN_CONTEXT_ORDER, SCAN_FORMAT_QR, "")
        with self.assertRaises(UnicodeEncodeError):
            encode_scan_result(1, SCAN_CONTEXT_ORDER, SCAN_FORMAT_QR, "病床")
        with self.assertRaises(ValueError):
            decode_scan_result(b"\x00\x01\x01\x01\x02A")
        with self.assertRaises(UnicodeDecodeError):
            decode_scan_result(b"\x00\x01\x01\x01\x01\xFF")
        with self.assertRaises(ValueError):
            decode_scan_result(b"\x00\x01\x01\x01\x01\x00")
        with self.assertRaises(ValueError):
            encode_scan_result(1, 9, SCAN_FORMAT_QR, "11")
        with self.assertRaises(ValueError):
            decode_scan_ack(b"\x00\x01\x09")

    def test_state_format_and_whitelist_are_all_required(self):
        self.assertTrue(scan_matches_task(2, SCAN_CONTEXT_ORDER, SCAN_FORMAT_QR, "31"))
        self.assertFalse(scan_matches_task(4, SCAN_CONTEXT_ORDER, SCAN_FORMAT_QR, "31"))
        self.assertFalse(
            scan_matches_task(2, SCAN_CONTEXT_ORDER, SCAN_FORMAT_CODE128, "31")
        )
        self.assertFalse(scan_matches_task(2, SCAN_CONTEXT_ORDER, SCAN_FORMAT_QR, "99"))
        self.assertTrue(
            scan_matches_task(
                4, SCAN_CONTEXT_BED1, SCAN_FORMAT_CODE128, "6906841121017"
            )
        )

    def test_consensus_requires_two_recent_hits_and_reset_rearms(self):
        consensus = ScanConsensus(required_hits=2, window_s=0.8)
        self.assertFalse(consensus.observe(SCAN_FORMAT_QR, "11", 1.0))
        self.assertTrue(consensus.observe(SCAN_FORMAT_QR, "11", 1.5))
        self.assertFalse(consensus.observe(SCAN_FORMAT_QR, "11", 1.6))
        consensus.reset()
        self.assertFalse(consensus.observe(SCAN_FORMAT_QR, "11", 2.0))
        self.assertTrue(consensus.observe(SCAN_FORMAT_QR, "11", 2.1))

    def test_consensus_drops_expired_hit(self):
        consensus = ScanConsensus(required_hits=2, window_s=0.8)
        self.assertFalse(consensus.observe(SCAN_FORMAT_CODE128, "6946522463487", 1.0))
        self.assertFalse(consensus.observe(SCAN_FORMAT_CODE128, "6946522463487", 2.0))


if __name__ == "__main__":
    unittest.main()
