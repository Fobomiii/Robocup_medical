"""Pure stall detection and geometry for collision-checked omni escape."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


Point2D = Tuple[float, float]


class BlockedIntentLatch:
    """Retain a proven collision-blocked motion vector through Nav2 recovery."""

    def __init__(
        self,
        *,
        hold_s: float,
        minimum_upstream_speed_m_s: float,
        maximum_safe_speed_m_s: float,
    ) -> None:
        values = (hold_s, minimum_upstream_speed_m_s, maximum_safe_speed_m_s)
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError("blocked-intent thresholds must be finite and non-negative")
        if hold_s <= 0.0:
            raise ValueError("blocked-intent hold must be positive")
        self.hold_s = hold_s
        self.minimum_upstream_speed_m_s = minimum_upstream_speed_m_s
        self.maximum_safe_speed_m_s = maximum_safe_speed_m_s
        self.vector: Optional[Point2D] = None
        self.latched_s: Optional[float] = None

    def reset(self) -> None:
        self.vector = None
        self.latched_s = None

    def update(
        self,
        *,
        now_s: float,
        upstream_x_m_s: float,
        upstream_y_m_s: float,
        safe_speed_m_s: float,
    ) -> bool:
        values = (now_s, upstream_x_m_s, upstream_y_m_s, safe_speed_m_s)
        if any(not math.isfinite(value) for value in values):
            self.reset()
            return False
        if self.latched_s is not None and now_s < self.latched_s:
            self.reset()
        upstream_speed_m_s = math.hypot(upstream_x_m_s, upstream_y_m_s)
        if (
            upstream_speed_m_s >= self.minimum_upstream_speed_m_s
            and safe_speed_m_s <= self.maximum_safe_speed_m_s
        ):
            self.vector = (upstream_x_m_s, upstream_y_m_s)
            self.latched_s = now_s
            return True
        return self.get(now_s) is not None

    def get(self, now_s: float) -> Optional[Point2D]:
        if self.vector is None or self.latched_s is None:
            return None
        if not math.isfinite(now_s) or not 0.0 <= now_s - self.latched_s <= self.hold_s:
            self.reset()
            return None
        return self.vector

    def age_s(self, now_s: float) -> Optional[float]:
        if self.get(now_s) is None or self.latched_s is None:
            return None
        return max(0.0, now_s - self.latched_s)


@dataclass(frozen=True)
class StallDecision:
    ready: bool
    active: bool
    elapsed_s: float
    displacement_m: float
    reason: str


class StallDetector:
    """Require uninterrupted motion intent and no physical progress."""

    def __init__(
        self,
        *,
        duration_s: float,
        minimum_upstream_speed_m_s: float,
        maximum_safe_speed_m_s: float,
        maximum_measured_speed_m_s: float,
        maximum_displacement_m: float,
    ) -> None:
        values = (
            duration_s,
            minimum_upstream_speed_m_s,
            maximum_safe_speed_m_s,
            maximum_measured_speed_m_s,
            maximum_displacement_m,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError("stall thresholds must be finite and non-negative")
        if duration_s <= 0.0:
            raise ValueError("stall duration must be positive")
        self.duration_s = duration_s
        self.minimum_upstream_speed_m_s = minimum_upstream_speed_m_s
        self.maximum_safe_speed_m_s = maximum_safe_speed_m_s
        self.maximum_measured_speed_m_s = maximum_measured_speed_m_s
        self.maximum_displacement_m = maximum_displacement_m
        self.started_s: Optional[float] = None
        self.started_xy: Optional[Point2D] = None

    def reset(self) -> None:
        self.started_s = None
        self.started_xy = None

    def update(
        self,
        *,
        now_s: float,
        eligible: bool,
        upstream_speed_m_s: float,
        safe_speed_m_s: float,
        measured_speed_m_s: float,
        position_x_m: float,
        position_y_m: float,
    ) -> StallDecision:
        samples = (
            now_s,
            upstream_speed_m_s,
            safe_speed_m_s,
            measured_speed_m_s,
            position_x_m,
            position_y_m,
        )
        if any(not math.isfinite(value) for value in samples):
            self.reset()
            return StallDecision(False, False, 0.0, 0.0, "non_finite_input")
        if not eligible:
            self.reset()
            return StallDecision(False, False, 0.0, 0.0, "guard_not_ready")
        if upstream_speed_m_s < self.minimum_upstream_speed_m_s:
            self.reset()
            return StallDecision(False, False, 0.0, 0.0, "no_motion_intent")
        if safe_speed_m_s > self.maximum_safe_speed_m_s:
            self.reset()
            return StallDecision(False, False, 0.0, 0.0, "safe_command_nonzero")
        if measured_speed_m_s > self.maximum_measured_speed_m_s:
            self.reset()
            return StallDecision(False, False, 0.0, 0.0, "robot_moving")

        if (
            self.started_s is None
            or self.started_xy is None
            or now_s < self.started_s
        ):
            self.started_s = now_s
            self.started_xy = (position_x_m, position_y_m)
            return StallDecision(False, True, 0.0, 0.0, "stall_timing")

        displacement_m = math.hypot(
            position_x_m - self.started_xy[0],
            position_y_m - self.started_xy[1],
        )
        if displacement_m > self.maximum_displacement_m:
            self.reset()
            return StallDecision(
                False, False, 0.0, displacement_m, "physical_progress"
            )
        elapsed_s = max(0.0, now_s - self.started_s)
        return StallDecision(
            elapsed_s >= self.duration_s,
            True,
            elapsed_s,
            displacement_m,
            "stall_confirmed" if elapsed_s >= self.duration_s else "stall_timing",
        )


@dataclass(frozen=True)
class EscapeDirectionDecision:
    valid: bool
    direction_x: float
    direction_y: float
    angle_rad: float
    score: float
    current_clearance_m: Optional[float]
    final_clearance_m: Optional[float]
    minimum_clearance_m: Optional[float]
    nearby_point_count: int
    rejected_direction_count: int
    reason: str


def _points_array(points: Iterable[Point2D]) -> np.ndarray:
    array = np.asarray(points, dtype=np.float64)
    if array.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError("points must have shape (N, 2)")
    return array[np.isfinite(array).all(axis=1)]


def _validate_geometry(
    footprint_radius_m: float,
    travel_distance_m: float,
    clearance_margin_m: float,
    evidence_min_points: int,
    max_collision_points: int,
) -> None:
    values = (footprint_radius_m, travel_distance_m, clearance_margin_m)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("escape geometry values must be finite")
    if footprint_radius_m <= 0.0:
        raise ValueError("footprint radius must be positive")
    if travel_distance_m <= 0.0:
        raise ValueError("travel distance must be positive")
    if clearance_margin_m < 0.0:
        raise ValueError("clearance margin must be non-negative")
    if evidence_min_points <= 0:
        raise ValueError("evidence point count must be positive")
    if max_collision_points < 0:
        raise ValueError("maximum collision point count must be non-negative")


def supported_clearance(
    points: Iterable[Point2D],
    center_x: float,
    center_y: float,
    footprint_radius_m: float,
    evidence_min_points: int,
) -> Optional[float]:
    """Return clearance supported by a cluster rather than one lidar return."""

    array = _points_array(points)
    if len(array) < evidence_min_points:
        return None
    offsets = array - np.array((center_x, center_y), dtype=np.float64)
    distances = np.hypot(offsets[:, 0], offsets[:, 1])
    kth_distance = float(
        np.partition(distances, evidence_min_points - 1)[evidence_min_points - 1]
    )
    return kth_distance - footprint_radius_m


def direction_is_swept_clear(
    points: Iterable[Point2D],
    direction_x: float,
    direction_y: float,
    *,
    footprint_radius_m: float,
    travel_distance_m: float,
    clearance_margin_m: float,
    max_collision_points: int,
    away_projection_epsilon_m: float = 0.005,
) -> bool:
    """Check the complete translated footprint capsule for supported contacts."""

    _validate_geometry(
        footprint_radius_m,
        travel_distance_m,
        clearance_margin_m,
        1,
        max_collision_points,
    )
    array = _points_array(points)
    if len(array) == 0:
        return True
    direction_norm = math.hypot(direction_x, direction_y)
    if not math.isfinite(direction_norm) or direction_norm <= 1.0e-9:
        return False
    unit = np.array(
        (direction_x / direction_norm, direction_y / direction_norm),
        dtype=np.float64,
    )
    guarded_radius_m = footprint_radius_m + clearance_margin_m
    ranges = np.hypot(array[:, 0], array[:, 1])
    initially_close = ranges <= guarded_radius_m
    projections = array @ unit

    moving_toward_existing = initially_close & (
        projections > away_projection_epsilon_m
    )
    if int(np.count_nonzero(moving_toward_existing)) > max_collision_points:
        return False

    outside = ~initially_close
    clamped = np.clip(projections, 0.0, travel_distance_m)
    closest = array - clamped[:, np.newaxis] * unit
    capsule_distance = np.hypot(closest[:, 0], closest[:, 1])
    new_contacts = (
        outside
        & (projections > 0.0)
        & (capsule_distance <= guarded_radius_m)
    )
    return int(np.count_nonzero(new_contacts)) <= max_collision_points


def path_length(points: Sequence[Point2D]) -> float:
    total_m = 0.0
    previous: Optional[Point2D] = None
    for raw_point in points:
        point = (float(raw_point[0]), float(raw_point[1]))
        if not math.isfinite(point[0]) or not math.isfinite(point[1]):
            continue
        if previous is not None:
            total_m += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
    return total_m


def choose_escape_direction(
    points: Iterable[Point2D],
    preferred_x: float,
    preferred_y: float,
    *,
    footprint_radius_m: float = 0.265,
    travel_distance_m: float = 0.30,
    clearance_margin_m: float = 0.03,
    minimum_clearance_gain_m: float = 0.04,
    clearance_regression_tolerance_m: float = 0.015,
    sample_step_m: float = 0.05,
    direction_samples: int = 16,
    evidence_min_points: int = 8,
    max_collision_points: int = 3,
    nearby_radius_m: float = 0.75,
    preference_weight: float = 0.30,
) -> EscapeDirectionDecision:
    """Choose a clear direction, biased toward the current path-following intent."""

    _validate_geometry(
        footprint_radius_m,
        travel_distance_m,
        clearance_margin_m,
        evidence_min_points,
        max_collision_points,
    )
    if direction_samples < 4:
        raise ValueError("at least four escape directions are required")
    if not math.isfinite(sample_step_m) or sample_step_m <= 0.0:
        raise ValueError("sample step must be positive")
    if not math.isfinite(nearby_radius_m) or nearby_radius_m <= 0.0:
        raise ValueError("nearby radius must be positive")
    if minimum_clearance_gain_m < 0.0:
        raise ValueError("minimum clearance gain must be non-negative")
    if clearance_regression_tolerance_m < 0.0:
        raise ValueError("clearance regression tolerance must be non-negative")

    array = _points_array(points)
    if len(array) < evidence_min_points:
        return EscapeDirectionDecision(
            False,
            0.0,
            0.0,
            0.0,
            -math.inf,
            None,
            None,
            None,
            len(array),
            0,
            "insufficient_points",
        )
    ranges = np.hypot(array[:, 0], array[:, 1])
    nearby_count = int(np.count_nonzero(ranges <= nearby_radius_m))
    if nearby_count < evidence_min_points:
        return EscapeDirectionDecision(
            False,
            0.0,
            0.0,
            0.0,
            -math.inf,
            None,
            None,
            None,
            nearby_count,
            0,
            "no_nearby_obstacle",
        )

    current_clearance = supported_clearance(
        array, 0.0, 0.0, footprint_radius_m, evidence_min_points
    )
    assert current_clearance is not None

    preferred_norm = math.hypot(preferred_x, preferred_y)
    if math.isfinite(preferred_norm) and preferred_norm > 1.0e-9:
        preferred = np.array(
            (preferred_x / preferred_norm, preferred_y / preferred_norm),
            dtype=np.float64,
        )
    else:
        nearby = array[ranges <= nearby_radius_m]
        nearby_ranges = np.maximum(
            np.hypot(nearby[:, 0], nearby[:, 1]), 1.0e-3
        )
        repulsion = np.sum(-nearby / nearby_ranges[:, np.newaxis] ** 2, axis=0)
        repulsion_norm = float(np.hypot(repulsion[0], repulsion[1]))
        preferred = (
            repulsion / repulsion_norm
            if repulsion_norm > 1.0e-9
            else np.array((0.0, 0.0), dtype=np.float64)
        )

    sample_count = max(1, int(math.ceil(travel_distance_m / sample_step_m)))
    distances = np.linspace(
        travel_distance_m / sample_count,
        travel_distance_m,
        sample_count,
    )
    best = None
    rejected_count = 0
    for direction_index in range(direction_samples):
        angle_rad = 2.0 * math.pi * direction_index / direction_samples
        unit = np.array((math.cos(angle_rad), math.sin(angle_rad)))
        if not direction_is_swept_clear(
            array,
            float(unit[0]),
            float(unit[1]),
            footprint_radius_m=footprint_radius_m,
            travel_distance_m=travel_distance_m,
            clearance_margin_m=clearance_margin_m,
            max_collision_points=max_collision_points,
        ):
            rejected_count += 1
            continue

        clearances = [
            supported_clearance(
                array,
                float(distance_m * unit[0]),
                float(distance_m * unit[1]),
                footprint_radius_m,
                evidence_min_points,
            )
            for distance_m in distances
        ]
        finite_clearances = [value for value in clearances if value is not None]
        if not finite_clearances:
            rejected_count += 1
            continue
        final_clearance = finite_clearances[-1]
        minimum_clearance = min(finite_clearances)
        clearance_gain = final_clearance - current_clearance
        if clearance_gain < minimum_clearance_gain_m:
            rejected_count += 1
            continue
        if minimum_clearance < current_clearance - clearance_regression_tolerance_m:
            rejected_count += 1
            continue

        alignment = float(unit @ preferred)
        score = final_clearance + 0.35 * clearance_gain + preference_weight * alignment
        candidate = (
            score,
            float(unit[0]),
            float(unit[1]),
            angle_rad,
            final_clearance,
            minimum_clearance,
        )
        if best is None or candidate[0] > best[0]:
            best = candidate

    if best is None:
        return EscapeDirectionDecision(
            False,
            0.0,
            0.0,
            0.0,
            -math.inf,
            current_clearance,
            None,
            None,
            nearby_count,
            rejected_count,
            "no_safe_direction",
        )

    return EscapeDirectionDecision(
        True,
        best[1],
        best[2],
        best[3],
        best[0],
        current_clearance,
        best[4],
        best[5],
        nearby_count,
        rejected_count,
        "safe_direction",
    )
