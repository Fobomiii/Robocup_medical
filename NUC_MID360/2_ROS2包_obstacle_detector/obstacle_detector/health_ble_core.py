"""Binary protocol and dashboard helpers for ESP32-S3 vital signs."""

import json
import struct
from dataclasses import dataclass
from typing import Iterable, Optional


HEALTH_PACKET_VERSION = 1
HEALTH_PACKET_FORMAT = "<BHHhBIH"
HEALTH_PACKET_SIZE = struct.calcsize(HEALTH_PACKET_FORMAT)
HEART_RATE_VALID = 0x01
TEMPERATURE_VALID = 0x02


def ble_advertisement_matches(
    configured_name: str,
    configured_address: str,
    service_uuid: str,
    candidate_name: str,
    candidate_address: str,
    advertised_service_uuids: Iterable[str],
) -> bool:
    if configured_address:
        return candidate_address.casefold() == configured_address.casefold()
    advertised = {str(uuid).lower() for uuid in advertised_service_uuids}
    return candidate_name == configured_name or service_uuid.lower() in advertised


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class HealthSample:
    sequence: int
    heart_rate_bpm: Optional[int]
    temperature_c: Optional[float]
    uptime_ms: int


def decode_health_packet(packet: bytes) -> Optional[HealthSample]:
    if len(packet) != HEALTH_PACKET_SIZE:
        return None
    version, sequence, bpm, temperature_centi, flags, uptime_ms, received_crc = (
        struct.unpack(HEALTH_PACKET_FORMAT, packet)
    )
    if version != HEALTH_PACKET_VERSION or crc16_ccitt(packet[:-2]) != received_crc:
        return None

    heart_rate = bpm if flags & HEART_RATE_VALID and 30 <= bpm <= 240 else None
    temperature = (
        temperature_centi / 100.0
        if flags & TEMPERATURE_VALID and 0 <= temperature_centi <= 6000
        else None
    )
    return HealthSample(sequence, heart_rate, temperature, uptime_ms)


def parse_health_status(payload: str) -> Optional[dict]:
    try:
        data = json.loads(payload)
        connected = bool(data["connected"])
        ble_link = bool(data.get("ble_link", connected))
        phase = str(data.get("phase", ""))
        heart_rate = data.get("heart_rate_bpm")
        temperature = data.get("temperature_c")
        if heart_rate is not None:
            heart_rate = int(heart_rate)
        if temperature is not None:
            temperature = float(temperature)
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None

    return {
        "connected": connected,
        "ble_link": ble_link,
        "phase": phase,
        "heart_rate_bpm": heart_rate if connected else None,
        "temperature_c": temperature if connected else None,
    }
