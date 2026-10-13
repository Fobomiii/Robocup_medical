"""Pure NumPy SE(2) motion interpolation and point-cloud deskew."""

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class MotionSample:
    stamp_ns: int
    x: float
    y: float
    yaw: float
    body_vx: float
    body_vy: float
    yaw_rate: float


def _world_velocity(sample: MotionSample) -> tuple[float, float]:
    cosine = math.cos(sample.yaw)
    sine = math.sin(sample.yaw)
    return (
        cosine * sample.body_vx - sine * sample.body_vy,
        sine * sample.body_vx + cosine * sample.body_vy,
    )


def _edge_pose(sample: MotionSample, query_ns: np.ndarray) -> np.ndarray:
    dt = (query_ns.astype(np.float64) - sample.stamp_ns) / 1.0e9
    midpoint_yaw = sample.yaw + 0.5 * sample.yaw_rate * dt
    world_vx = np.cos(midpoint_yaw) * sample.body_vx - np.sin(
        midpoint_yaw
    ) * sample.body_vy
    world_vy = np.sin(midpoint_yaw) * sample.body_vx + np.cos(
        midpoint_yaw
    ) * sample.body_vy
    return np.column_stack(
        (
            sample.x + world_vx * dt,
            sample.y + world_vy * dt,
            sample.yaw + sample.yaw_rate * dt,
        )
    )


def interpolate_motion(
    samples: Sequence[MotionSample],
    query_ns: np.ndarray,
    max_extrapolation_ns: int,
) -> np.ndarray | None:
    """Interpolate pose using wheel-velocity Hermite curves and yaw history."""
    if not samples:
        return None
    query_ns = np.asarray(query_ns, dtype=np.int64)
    if query_ns.ndim != 1 or query_ns.size == 0:
        return None

    ordered: list[MotionSample] = []
    for sample in sorted(samples, key=lambda item: item.stamp_ns):
        if ordered and sample.stamp_ns == ordered[-1].stamp_ns:
            ordered[-1] = sample
        else:
            ordered.append(sample)
    stamps = np.asarray([sample.stamp_ns for sample in ordered], dtype=np.int64)
    if (
        int(np.min(query_ns)) < int(stamps[0]) - int(max_extrapolation_ns)
        or int(np.max(query_ns)) > int(stamps[-1]) + int(max_extrapolation_ns)
    ):
        return None

    result = np.empty((len(query_ns), 3), dtype=np.float64)
    before = query_ns < stamps[0]
    after = query_ns > stamps[-1]
    inside = ~(before | after)
    if np.any(before):
        result[before] = _edge_pose(ordered[0], query_ns[before])
    if np.any(after):
        result[after] = _edge_pose(ordered[-1], query_ns[after])

    if np.any(inside):
        inside_times = query_ns[inside]
        if len(ordered) == 1:
            result[inside] = _edge_pose(ordered[0], inside_times)
        else:
            right = np.searchsorted(stamps, inside_times, side="right")
            right = np.clip(right, 1, len(ordered) - 1)
            left = right - 1
            t0 = stamps[left].astype(np.float64)
            t1 = stamps[right].astype(np.float64)
            interval_ns = t1 - t0
            fraction = (inside_times.astype(np.float64) - t0) / interval_ns
            duration_s = interval_ns / 1.0e9

            x = np.asarray([sample.x for sample in ordered], dtype=np.float64)
            y = np.asarray([sample.y for sample in ordered], dtype=np.float64)
            yaw = np.unwrap(
                np.asarray([sample.yaw for sample in ordered], dtype=np.float64)
            )
            world_velocity = np.asarray(
                [_world_velocity(sample) for sample in ordered], dtype=np.float64
            )
            yaw_rate = np.asarray(
                [sample.yaw_rate for sample in ordered], dtype=np.float64
            )

            h00 = 2.0 * fraction**3 - 3.0 * fraction**2 + 1.0
            h10 = fraction**3 - 2.0 * fraction**2 + fraction
            h01 = -2.0 * fraction**3 + 3.0 * fraction**2
            h11 = fraction**3 - fraction**2
            result[inside, 0] = (
                h00 * x[left]
                + h10 * duration_s * world_velocity[left, 0]
                + h01 * x[right]
                + h11 * duration_s * world_velocity[right, 0]
            )
            result[inside, 1] = (
                h00 * y[left]
                + h10 * duration_s * world_velocity[left, 1]
                + h01 * y[right]
                + h11 * duration_s * world_velocity[right, 1]
            )
            result[inside, 2] = (
                h00 * yaw[left]
                + h10 * duration_s * yaw_rate[left]
                + h01 * yaw[right]
                + h11 * duration_s * yaw_rate[right]
            )

    result[:, 2] = np.arctan2(np.sin(result[:, 2]), np.cos(result[:, 2]))
    return result


def deskew_points(
    xyz_base: np.ndarray,
    point_time_ns: np.ndarray,
    samples: Sequence[MotionSample],
    reference_ns: int,
    max_extrapolation_ns: int,
) -> np.ndarray | None:
    """Express points captured at different times in base at reference time."""
    xyz_base = np.asarray(xyz_base, dtype=np.float64)
    point_time_ns = np.asarray(point_time_ns, dtype=np.int64)
    if xyz_base.ndim != 2 or xyz_base.shape[1] != 3:
        raise ValueError("xyz_base must be an Nx3 array")
    if len(xyz_base) != len(point_time_ns) or len(xyz_base) == 0:
        return None

    query = np.concatenate(
        (point_time_ns, np.asarray([reference_ns], dtype=np.int64))
    )
    poses = interpolate_motion(samples, query, max_extrapolation_ns)
    if poses is None:
        return None
    point_pose = poses[:-1]
    reference_pose = poses[-1]

    cosine = np.cos(point_pose[:, 2])
    sine = np.sin(point_pose[:, 2])
    world_x = point_pose[:, 0] + cosine * xyz_base[:, 0] - sine * xyz_base[:, 1]
    world_y = point_pose[:, 1] + sine * xyz_base[:, 0] + cosine * xyz_base[:, 1]

    delta_x = world_x - reference_pose[0]
    delta_y = world_y - reference_pose[1]
    reference_cosine = math.cos(reference_pose[2])
    reference_sine = math.sin(reference_pose[2])
    output = xyz_base.copy()
    output[:, 0] = reference_cosine * delta_x + reference_sine * delta_y
    output[:, 1] = -reference_sine * delta_x + reference_cosine * delta_y
    return output.astype(np.float32)
