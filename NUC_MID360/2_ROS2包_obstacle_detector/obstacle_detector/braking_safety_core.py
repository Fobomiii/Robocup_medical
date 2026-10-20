"""Pure geometry and braking-envelope helpers for shadow safety analysis."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Tuple


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class BrakingSafetyDecision:
    collision_distance_m: Optional[float]
    effective_distance_m: Optional[float]
    command_speed_m_s: float
    measured_speed_m_s: float
    stopping_distance_m: float
    speed_limit_m_s: float
    shadow_speed_m_s: float
    scale: float
    candidate_point_count: int
    would_limit: bool
    would_stop: bool
    reason: str


def stopping_distance(
    speed_m_s: float,
    braking_decel_m_s2: float,
    reaction_time_s: float,
) -> float:
    """Return reaction distance plus constant-deceleration stopping distance."""

    if not math.isfinite(speed_m_s) or speed_m_s < 0.0:
        raise ValueError("speed must be finite and non-negative")
    if not math.isfinite(braking_decel_m_s2) or braking_decel_m_s2 <= 0.0:
        raise ValueError("braking deceleration must be finite and positive")
    if not math.isfinite(reaction_time_s) or reaction_time_s < 0.0:
        raise ValueError("reaction time must be finite and non-negative")
    return (
        speed_m_s * reaction_time_s
        + speed_m_s * speed_m_s / (2.0 * braking_decel_m_s2)
    )


def braking_speed_limit(
    distance_m: float,
    braking_decel_m_s2: float,
    reaction_time_s: float,
    max_speed_m_s: float,
) -> float:
    """Invert the stopping-distance model for the safe speed at a distance."""

    if not math.isfinite(distance_m) or distance_m < 0.0:
        raise ValueError("distance must be finite and non-negative")
    if not math.isfinite(max_speed_m_s) or max_speed_m_s <= 0.0:
        raise ValueError("maximum speed must be finite and positive")
    if not math.isfinite(braking_decel_m_s2) or braking_decel_m_s2 <= 0.0:
        raise ValueError("braking deceleration must be finite and positive")
    if not math.isfinite(reaction_time_s) or reaction_time_s < 0.0:
        raise ValueError("reaction time must be finite and non-negative")

    reaction_velocity_m_s = braking_decel_m_s2 * reaction_time_s
    limit_m_s = (
        math.sqrt(
            reaction_velocity_m_s * reaction_velocity_m_s
            + 2.0 * braking_decel_m_s2 * distance_m
        )
        - reaction_velocity_m_s
    )
    return min(max_speed_m_s, max(0.0, limit_m_s))


def swept_footprint_collision_distance(
    points: Iterable[Point2D],
    direction_x: float,
    direction_y: float,
    footprint_radius_m: float,
    max_lookahead_m: float,
    min_points: int,
) -> tuple[Optional[float], int]:
    """Find the first supported point contact for a swept circular footprint."""

    if not math.isfinite(footprint_radius_m) or footprint_radius_m <= 0.0:
        raise ValueError("footprint radius must be finite and positive")
    if not math.isfinite(max_lookahead_m) or max_lookahead_m <= 0.0:
        raise ValueError("maximum lookahead must be finite and positive")
    if min_points <= 0:
        raise ValueError("minimum point count must be positive")

    direction_norm = math.hypot(direction_x, direction_y)
    if not math.isfinite(direction_norm) or direction_norm <= 1.0e-9:
        return None, 0
    unit_x = direction_x / direction_norm
    unit_y = direction_y / direction_norm
    radius_sq = footprint_radius_m * footprint_radius_m
    contacts: list[float] = []

    for point_x, point_y in points:
        if not math.isfinite(point_x) or not math.isfinite(point_y):
            continue
        point_range_sq = point_x * point_x + point_y * point_y
        if point_range_sq <= radius_sq:
            contacts.append(0.0)
            continue

        longitudinal_m = point_x * unit_x + point_y * unit_y
        lateral_m = abs(point_x * unit_y - point_y * unit_x)
        if longitudinal_m <= 0.0 or lateral_m > footprint_radius_m:
            continue
        contact_m = longitudinal_m - math.sqrt(
            max(0.0, radius_sq - lateral_m * lateral_m)
        )
        if 0.0 <= contact_m <= max_lookahead_m:
            contacts.append(contact_m)

    contacts.sort()
    if len(contacts) < min_points:
        return None, len(contacts)
    return contacts[min_points - 1], len(contacts)


def evaluate_braking_safety(
    points: Iterable[Point2D],
    command_vx_m_s: float,
    command_vy_m_s: float,
    measured_vx_m_s: float,
    measured_vy_m_s: float,
    *,
    braking_decel_m_s2: float = 2.5,
    reaction_time_s: float = 0.20,
    safety_margin_m: float = 0.15,
    footprint_radius_m: float = 0.245,
    max_speed_m_s: float = 3.0,
    min_points: int = 5,
    max_lookahead_m: float = 4.0,
    direction_deadband_m_s: float = 0.02,
) -> BrakingSafetyDecision:
    """Evaluate a continuous shadow speed ceiling without commanding motion."""

    velocities = (
        command_vx_m_s,
        command_vy_m_s,
        measured_vx_m_s,
        measured_vy_m_s,
    )
    if any(not math.isfinite(value) for value in velocities):
        raise ValueError("velocity inputs must be finite")
    if not math.isfinite(safety_margin_m) or safety_margin_m < 0.0:
        raise ValueError("safety margin must be finite and non-negative")
    if not math.isfinite(direction_deadband_m_s) or direction_deadband_m_s < 0.0:
        raise ValueError("direction deadband must be finite and non-negative")
    if not math.isfinite(footprint_radius_m) or footprint_radius_m <= 0.0:
        raise ValueError("footprint radius must be finite and positive")
    if not math.isfinite(max_lookahead_m) or max_lookahead_m <= 0.0:
        raise ValueError("maximum lookahead must be finite and positive")
    if not math.isfinite(max_speed_m_s) or max_speed_m_s <= 0.0:
        raise ValueError("maximum speed must be finite and positive")
    if min_points <= 0:
        raise ValueError("minimum point count must be positive")

    command_speed_m_s = math.hypot(command_vx_m_s, command_vy_m_s)
    measured_speed_m_s = math.hypot(measured_vx_m_s, measured_vy_m_s)
    stopping_distance_m = stopping_distance(
        measured_speed_m_s,
        braking_decel_m_s2,
        reaction_time_s,
    )
    if command_speed_m_s > direction_deadband_m_s:
        direction_x = command_vx_m_s
        direction_y = command_vy_m_s
    elif measured_speed_m_s > direction_deadband_m_s:
        direction_x = measured_vx_m_s
        direction_y = measured_vy_m_s
    else:
        return BrakingSafetyDecision(
            collision_distance_m=None,
            effective_distance_m=None,
            command_speed_m_s=command_speed_m_s,
            measured_speed_m_s=measured_speed_m_s,
            stopping_distance_m=stopping_distance_m,
            speed_limit_m_s=max_speed_m_s,
            shadow_speed_m_s=0.0,
            scale=1.0,
            candidate_point_count=0,
            would_limit=False,
            would_stop=False,
            reason="stationary",
        )

    collision_distance_m, candidate_point_count = (
        swept_footprint_collision_distance(
            points,
            direction_x,
            direction_y,
            footprint_radius_m,
            max_lookahead_m,
            min_points,
        )
    )
    if collision_distance_m is None:
        speed_limit_m_s = max_speed_m_s
        effective_distance_m = None
        reason = "no_supported_obstacle"
    else:
        effective_distance_m = max(0.0, collision_distance_m - safety_margin_m)
        speed_limit_m_s = braking_speed_limit(
            effective_distance_m,
            braking_decel_m_s2,
            reaction_time_s,
            max_speed_m_s,
        )
        reason = "braking_envelope"

    shadow_speed_m_s = min(command_speed_m_s, speed_limit_m_s)
    scale = (
        1.0
        if command_speed_m_s <= 1.0e-9
        else min(1.0, shadow_speed_m_s / command_speed_m_s)
    )
    would_limit = shadow_speed_m_s < command_speed_m_s - 1.0e-6
    would_stop = command_speed_m_s > 1.0e-6 and shadow_speed_m_s <= 1.0e-6
    return BrakingSafetyDecision(
        collision_distance_m=collision_distance_m,
        effective_distance_m=effective_distance_m,
        command_speed_m_s=command_speed_m_s,
        measured_speed_m_s=measured_speed_m_s,
        stopping_distance_m=stopping_distance_m,
        speed_limit_m_s=speed_limit_m_s,
        shadow_speed_m_s=shadow_speed_m_s,
        scale=scale,
        candidate_point_count=candidate_point_count,
        would_limit=would_limit,
        would_stop=would_stop,
        reason=reason,
    )
