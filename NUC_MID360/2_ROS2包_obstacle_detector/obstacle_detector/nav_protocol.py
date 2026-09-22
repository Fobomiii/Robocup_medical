"""Binary navigation protocol shared by the NUC tools.

Frame layout (big-endian payload fields):
    AA CC | version | type | seq | payload_len | payload | CRC16

CRC16 is CCITT-FALSE over version through the end of payload. The existing
A5 5A handshake and AA BB obstacle frame use different sync bytes and remain
compatible during migration.
"""

from dataclasses import dataclass
import math
import struct
from typing import Iterable, List


SYNC = b"\xAA\xCC"
VERSION = 1
MAX_PAYLOAD = 48

MSG_POSE = 0x10
MSG_GOAL_REQUEST = 0x11
MSG_NAV_STATUS = 0x12
MSG_STP23L = 0x13
MSG_SCAN_ACK = 0x14
MSG_PATH_BEGIN = 0x20
MSG_WAYPOINT = 0x21
MSG_PATH_COMMIT = 0x22
MSG_PATH_CANCEL = 0x23
MSG_HEARTBEAT = 0x30
MSG_VEL_CMD = 0x40
MSG_SCAN_RESULT = 0x41

SCAN_CONTEXT_ORDER = 1
SCAN_CONTEXT_BED1 = 2
SCAN_CONTEXT_BED3 = 3

SCAN_FORMAT_QR = 1
SCAN_FORMAT_CODE128 = 2

SCAN_ACK_ACCEPTED = 1
SCAN_ACK_WRONG_STATE = 2
SCAN_ACK_INVALID_CODE = 3
SCAN_CODE_MAX = 32

GOAL_NONE = 0
GOAL_HOME = 1
GOAL_NURSE = 2
GOAL_BED1 = 3
GOAL_BED3 = 4

NAV_IDLE = 0
NAV_WAIT_PATH = 1
NAV_FOLLOWING = 2
NAV_REACHED = 3
NAV_ERROR = 4


@dataclass(frozen=True)
class Frame:
    msg_type: int
    seq: int
    payload: bytes


@dataclass(frozen=True)
class PoseTelemetry:
    x_mm: int
    y_mm: int
    yaw_cdeg: int
    task_state: int
    nav_status: int
    path_id: int
    waypoint_index: int


@dataclass(frozen=True)
class GoalRequest:
    request_id: int
    goal_id: int


@dataclass(frozen=True)
class Stp23lTelemetry:
    """Three sensor-face distances in millimetres.

    Installation: A=right, B=front and C=left. ``valid_mask`` uses bits
    0, 1 and 2 respectively; a zero distance must never be treated as a wall.
    """

    a_mm: int
    b_mm: int
    c_mm: int
    valid_mask: int


@dataclass(frozen=True)
class ScanResult:
    scan_id: int
    context: int
    format: int
    value: str


@dataclass(frozen=True)
class ScanAck:
    scan_id: int
    status: int


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def clamp_i16(value: int) -> int:
    return max(-32768, min(32767, int(value)))


def encode_velocity(mps_x: float, mps_y: float, rps_yaw: float, stamp_cs: int) -> bytes:
    """Body-frame velocity in millimetres/s and centi-degrees/s.

    ROS convention: +vx forward, +vy left, +w counter-clockwise. The STM32
    chassis layer applies the mecanum mixing and the robot's own sign rules.
    ``stamp_cs`` is a centisecond timestamp used to discard stale commands.
    """
    return struct.pack(
        ">hhhH",
        clamp_i16(round(mps_x * 1000.0)),
        clamp_i16(round(mps_y * 1000.0)),
        clamp_i16(round(rps_yaw * 18000.0 / math.pi)),  # rad/s -> centi-deg/s
        int(stamp_cs) & 0xFFFF,
    )


def encode_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too large: {len(payload)} > {MAX_PAYLOAD}")
    body = bytes((VERSION, msg_type & 0xFF, seq & 0xFF, len(payload))) + payload
    return SYNC + body + struct.pack(">H", crc16_ccitt(body))


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer.extend(data)
        frames: List[Frame] = []

        while True:
            sync_index = self._buffer.find(SYNC)
            if sync_index < 0:
                if self._buffer[-1:] == SYNC[:1]:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break
            if sync_index:
                del self._buffer[:sync_index]
            if len(self._buffer) < 8:
                break

            version, msg_type, seq, payload_len = self._buffer[2:6]
            if version != VERSION or payload_len > MAX_PAYLOAD:
                del self._buffer[0]
                continue

            frame_len = 2 + 4 + payload_len + 2
            if len(self._buffer) < frame_len:
                break

            candidate = bytes(self._buffer[:frame_len])
            body = candidate[2:-2]
            expected_crc = struct.unpack(">H", candidate[-2:])[0]
            if crc16_ccitt(body) == expected_crc:
                frames.append(Frame(msg_type, seq, candidate[6:-2]))
                del self._buffer[:frame_len]
            else:
                del self._buffer[0]

        return frames


def decode_pose(payload: bytes) -> PoseTelemetry:
    if len(payload) != 15:
        raise ValueError(f"POSE payload length is {len(payload)}, expected 15")
    return PoseTelemetry(*struct.unpack(">iihBBHB", payload))


def decode_goal_request(payload: bytes) -> GoalRequest:
    if len(payload) != 3:
        raise ValueError(f"GOAL_REQUEST payload length is {len(payload)}, expected 3")
    return GoalRequest(*struct.unpack(">HB", payload))


def decode_stp23l(payload: bytes) -> Stp23lTelemetry:
    if len(payload) != 7:
        raise ValueError(f"STP23L payload length is {len(payload)}, expected 7")
    return Stp23lTelemetry(*struct.unpack(">HHHB", payload))


def encode_scan_result(scan_id: int, context: int, format: int, value: str) -> bytes:
    if context not in {SCAN_CONTEXT_ORDER, SCAN_CONTEXT_BED1, SCAN_CONTEXT_BED3}:
        raise ValueError(f"invalid scan context: {context}")
    if format not in {SCAN_FORMAT_QR, SCAN_FORMAT_CODE128}:
        raise ValueError(f"invalid scan format: {format}")
    encoded = value.encode("ascii", "strict")
    if not encoded or len(encoded) >= SCAN_CODE_MAX:
        raise ValueError(f"scan value length must be 1..{SCAN_CODE_MAX - 1}")
    if any(byte < 0x20 or byte > 0x7E for byte in encoded):
        raise ValueError("scan value contains a control character")
    return struct.pack(">HBBB", scan_id & 0xFFFF, context, format, len(encoded)) + encoded


def decode_scan_result(payload: bytes) -> ScanResult:
    if len(payload) < 6:
        raise ValueError("SCAN_RESULT payload is too short")
    scan_id, context, format, value_len = struct.unpack(">HBBB", payload[:5])
    if value_len == 0 or value_len >= SCAN_CODE_MAX or len(payload) != 5 + value_len:
        raise ValueError("SCAN_RESULT value length is invalid")
    if context not in {SCAN_CONTEXT_ORDER, SCAN_CONTEXT_BED1, SCAN_CONTEXT_BED3}:
        raise ValueError(f"invalid scan context: {context}")
    if format not in {SCAN_FORMAT_QR, SCAN_FORMAT_CODE128}:
        raise ValueError(f"invalid scan format: {format}")
    value = payload[5:].decode("ascii", "strict")
    if any(ord(character) < 0x20 or ord(character) > 0x7E for character in value):
        raise ValueError("SCAN_RESULT value contains a control character")
    return ScanResult(scan_id, context, format, value)


def encode_scan_ack(scan_id: int, status: int) -> bytes:
    if status not in {SCAN_ACK_ACCEPTED, SCAN_ACK_WRONG_STATE, SCAN_ACK_INVALID_CODE}:
        raise ValueError(f"invalid scan ACK status: {status}")
    return struct.pack(">HB", scan_id & 0xFFFF, status & 0xFF)


def decode_scan_ack(payload: bytes) -> ScanAck:
    if len(payload) != 3:
        raise ValueError(f"SCAN_ACK payload length is {len(payload)}, expected 3")
    ack = ScanAck(*struct.unpack(">HB", payload))
    if ack.status not in {SCAN_ACK_ACCEPTED, SCAN_ACK_WRONG_STATE, SCAN_ACK_INVALID_CODE}:
        raise ValueError(f"invalid scan ACK status: {ack.status}")
    return ack


def encode_path_begin(path_id: int, request_id: int, goal_id: int, count: int) -> bytes:
    return struct.pack(">HHBB", path_id, request_id, goal_id, count)


def encode_waypoint(
    path_id: int,
    index: int,
    x_mm: int,
    y_mm: int,
    yaw_cdeg: int,
    speed_mm_s: int,
) -> bytes:
    return struct.pack(">HBiihH", path_id, index, x_mm, y_mm, yaw_cdeg, speed_mm_s)


def encode_nav_status(request_id: int, goal_id: int, status: int) -> bytes:
    return struct.pack(">HBB", request_id, goal_id, status)


def encode_path_commit(path_id: int, count: int) -> bytes:
    return struct.pack(">HB", path_id, count)


def encode_heartbeat(counter: int) -> bytes:
    return struct.pack(">I", counter & 0xFFFFFFFF)


def make_path_frames(
    path_id: int,
    request_id: int,
    goal_id: int,
    waypoints: Iterable,
    first_seq: int,
) -> List[bytes]:
    points = list(waypoints)
    if not points or len(points) > 16:
        raise ValueError("a path must contain 1..16 waypoints")

    seq = first_seq & 0xFF
    frames = [
        encode_frame(
            MSG_PATH_BEGIN,
            seq,
            encode_path_begin(path_id, request_id, goal_id, len(points)),
        )
    ]
    for index, point in enumerate(points):
        seq = (seq + 1) & 0xFF
        frames.append(
            encode_frame(
                MSG_WAYPOINT,
                seq,
                encode_waypoint(
                    path_id,
                    index,
                    int(round(point.x_mm)),
                    int(round(point.y_mm)),
                    int(round(point.yaw_deg * 100.0)),
                    int(round(point.speed_mm_s)),
                ),
            )
        )
    seq = (seq + 1) & 0xFF
    frames.append(encode_frame(MSG_PATH_COMMIT, seq, encode_path_commit(path_id, len(points))))
    return frames
