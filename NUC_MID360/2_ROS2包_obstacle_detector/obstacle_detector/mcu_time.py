"""Convert STM32 tick timestamps into ROS time without clock synchronisation."""

import math
from dataclasses import dataclass


UINT32_MASK = 0xFFFFFFFF


def elapsed_u32_ms(newer_ms: int, older_ms: int) -> int:
    """Unsigned 32-bit elapsed milliseconds, including HAL tick wraparound."""
    return (int(newer_ms) - int(older_ms)) & UINT32_MASK


def sample_ros_time_ns(
    sample_ms: int,
    tx_stamp_ms: int,
    received_ros_ns: int,
    max_sample_age_ms: int = 1000,
) -> int | None:
    """Map an MCU sample tick to ROS time using age inside one telemetry frame."""
    if sample_ms <= 0 or tx_stamp_ms <= 0 or received_ros_ns <= 0:
        return None
    age_ms = elapsed_u32_ms(tx_stamp_ms, sample_ms)
    if age_ms > max_sample_age_ms:
        return None
    return int(received_ros_ns) - age_ms * 1_000_000


@dataclass(frozen=True)
class AlignedPose2D:
    x: float
    y: float
    yaw: float
    stamp_ns: int


def align_pose_measurements(
    x: float,
    y: float,
    yaw: float,
    ops_stamp_ns: int,
    yaw_stamp_ns: int,
    body_vx: float,
    body_vy: float,
    yaw_rate: float,
) -> AlignedPose2D:
    """Propagate asynchronous OPS XY and HWT yaw to their newer timestamp."""
    reference_ns = max(int(ops_stamp_ns), int(yaw_stamp_ns))
    yaw_dt = (reference_ns - int(yaw_stamp_ns)) / 1.0e9
    reference_yaw = yaw + yaw_rate * yaw_dt
    position_dt = (reference_ns - int(ops_stamp_ns)) / 1.0e9
    if position_dt > 0.0:
        yaw_at_ops = yaw + yaw_rate * (
            (int(ops_stamp_ns) - int(yaw_stamp_ns)) / 1.0e9
        )
        average_yaw = yaw_at_ops + 0.5 * yaw_rate * position_dt
        world_vx = math.cos(average_yaw) * body_vx - math.sin(average_yaw) * body_vy
        world_vy = math.sin(average_yaw) * body_vx + math.cos(average_yaw) * body_vy
        x += world_vx * position_dt
        y += world_vy * position_dt
    reference_yaw = math.atan2(math.sin(reference_yaw), math.cos(reference_yaw))
    return AlignedPose2D(x, y, reference_yaw, reference_ns)
