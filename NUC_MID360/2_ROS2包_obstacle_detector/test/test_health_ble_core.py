import json
import struct
import unittest

from obstacle_detector.health_ble_core import (
    HEALTH_PACKET_FORMAT,
    HEART_RATE_VALID,
    TEMPERATURE_VALID,
    ble_advertisement_matches,
    crc16_ccitt,
    decode_health_packet,
    parse_health_status,
)


def packet(sequence=7, bpm=76, temperature_centi=3665, flags=3, uptime_ms=12345):
    prefix = struct.pack(
        "<BHHhBI", 1, sequence, bpm, temperature_centi, flags, uptime_ms
    )
    return prefix + struct.pack("<H", crc16_ccitt(prefix))


class HealthBleCoreTest(unittest.TestCase):
    def test_matches_name_address_or_service_uuid(self):
        service_uuid = "7d2e1000-5f5b-4f4b-9c61-7f1e9d5a0001"
        self.assertTrue(
            ble_advertisement_matches(
                "MedicalVitals-S3", "", service_uuid,
                "MedicalVitals-S3", "AA:BB", [],
            )
        )
        self.assertTrue(
            ble_advertisement_matches(
                "MedicalVitals-S3", "", service_uuid,
                "", "AA:BB", [service_uuid.upper()],
            )
        )
        self.assertTrue(
            ble_advertisement_matches(
                "MedicalVitals-S3", "aa:bb", service_uuid,
                "different", "AA:BB", [],
            )
        )
        self.assertFalse(
            ble_advertisement_matches(
                "MedicalVitals-S3", "AA:BB", service_uuid,
                "MedicalVitals-S3", "CC:DD", [service_uuid],
            )
        )

    def test_decodes_valid_packet(self):
        sample = decode_health_packet(packet())
        self.assertEqual(sample.sequence, 7)
        self.assertEqual(sample.heart_rate_bpm, 76)
        self.assertAlmostEqual(sample.temperature_c, 36.65)
        self.assertEqual(sample.uptime_ms, 12345)

    def test_respects_validity_flags(self):
        sample = decode_health_packet(packet(flags=0))
        self.assertIsNone(sample.heart_rate_bpm)
        self.assertIsNone(sample.temperature_c)

    def test_rejects_bad_crc_length_and_version(self):
        damaged = bytearray(packet())
        damaged[3] ^= 0x40
        self.assertIsNone(decode_health_packet(bytes(damaged)))
        self.assertIsNone(decode_health_packet(packet()[:-1]))
        fields = list(struct.unpack(HEALTH_PACKET_FORMAT, packet()))
        fields[0] = 2
        invalid_version = struct.pack(HEALTH_PACKET_FORMAT, *fields)
        self.assertIsNone(decode_health_packet(invalid_version))

    def test_parses_dashboard_status(self):
        parsed = parse_health_status(
            json.dumps(
                {
                    "connected": True,
                    "ble_link": True,
                    "phase": "connected",
                    "heart_rate_bpm": 78,
                    "temperature_c": 36.7,
                }
            )
        )
        self.assertEqual(parsed["heart_rate_bpm"], 78)
        self.assertAlmostEqual(parsed["temperature_c"], 36.7)
        self.assertTrue(parsed["ble_link"])
        self.assertEqual(parsed["phase"], "connected")
        offline = parse_health_status(
            '{"connected":false,"heart_rate_bpm":88,"temperature_c":37.1}'
        )
        self.assertIsNone(offline["heart_rate_bpm"])
        self.assertIsNone(offline["temperature_c"])

    def test_rejects_invalid_dashboard_status(self):
        self.assertIsNone(parse_health_status("bad-json"))
        self.assertIsNone(parse_health_status('{"temperature_c":36.5}'))


if __name__ == "__main__":
    unittest.main()
