"""Pure functions for the home-only approach speed envelope."""

from __future__ import annotations

import math
from typing import Optional, Tuple


def distance_to_home(
    x_m: float,
    y_m: float,
    home_x_m: float = 0.0,
    home_y_m: float = 0.0,
) -> float:
    """Return planar distance from the current pose to the home goal."""

    return math.hypot(float(x_m) - float(home_x_m), float(y_m) - float(home_y_m))


def approach_speed_limit(
    distance_m: float,
    *,
    max_speed_m_s: float = 3.0,
    soft_decel_m_s2: float = 1.425,
    terminal_speed_m_s: float = 0.10,
    terminal_distance_m: float = 0.10,
    decel_start_distance_m: Optional[float] = None,
) -> float:
    """Calculate a continuous speed limit for the remaining goal distance.

    When ``decel_start_distance_m`` is provided, the envelope remains at full
    speed up to that distance and reaches ``terminal_speed_m_s`` at the
    terminal radius. Otherwise, ``soft_decel_m_s2`` defines the envelope.
    """

    max_speed = max(0.0, float(max_speed_m_s))
    soft_decel = max(0.0, float(soft_decel_m_s2))
    terminal_speed = max(0.0, float(terminal_speed_m_s))
    terminal_distance = max(0.0, float(terminal_distance_m))
    distance = max(0.0, float(distance_m))

    if decel_start_distance_m is not None:
        start_distance = max(terminal_distance, float(decel_start_distance_m))
        if distance >= start_distance:
            return max_speed
        braking_distance = start_distance - terminal_distance
        if braking_distance <= 1.0e-9:
            return min(max_speed, terminal_speed)
        remaining = max(distance - terminal_distance, 0.0)
        speed_squared = terminal_speed * terminal_speed + (
            max_speed * max_speed - terminal_speed * terminal_speed
        ) * remaining / braking_distance
        envelope = math.sqrt(max(0.0, speed_squared))
        return min(max_speed, max(terminal_speed, envelope))

    if soft_decel == 0.0:
        return min(max_speed, terminal_speed if distance <= terminal_distance else max_speed)

    remaining = max(distance - terminal_distance, 0.0)
    envelope = math.sqrt(terminal_speed * terminal_speed + 2.0 * soft_decel * remaining)
    return min(max_speed, max(terminal_speed, envelope))


def scale_planar_velocity(
    vx_m_s: float,
    vy_m_s: float,
    speed_limit_m_s: float,
) -> Tuple[float, float]:
    """Scale a planar velocity to a limit while preserving its direction."""

    vx = float(vx_m_s)
    vy = float(vy_m_s)
    limit = max(0.0, float(speed_limit_m_s))
    speed = math.hypot(vx, vy)
    if speed <= limit or speed <= 1.0e-9:
        return vx, vy
    scale = limit / speed
    return vx * scale, vy * scale


def body_to_world_velocity(
    forward_m_s: float,
    left_m_s: float,
    yaw_rad: float,
) -> Tuple[float, float]:
    """Rotate a ROS base-frame velocity into the map frame."""

    cosine = math.cos(float(yaw_rad))
    sine = math.sin(float(yaw_rad))
    return (
        cosine * float(forward_m_s) - sine * float(left_m_s),
        sine * float(forward_m_s) + cosine * float(left_m_s),
    )


def world_to_body_velocity(
    world_vx_m_s: float,
    world_vy_m_s: float,
    yaw_rad: float,
) -> Tuple[float, float]:
    """Rotate a map-frame velocity into the ROS base frame."""

    cosine = math.cos(float(yaw_rad))
    sine = math.sin(float(yaw_rad))
    return (
        cosine * float(world_vx_m_s) + sine * float(world_vy_m_s),
        -sine * float(world_vx_m_s) + cosine * float(world_vy_m_s),
    )


def limit_velocity_toward_boundary(
    command_m_s: float,
    measured_m_s: Optional[float],
    position_m: float,
    boundary_m: float,
    toward_sign: float,
    *,
    braking_decel_m_s2: float,
    reaction_time_s: float,
    safety_margin_m: float,
    max_speed_m_s: float,
) -> Tuple[float, float, float]:
    """Limit only motion toward one axis-aligned goal boundary.

    ``toward_sign`` is fixed by field geometry rather than recomputed after an
    overshoot.  A positive value means increasing coordinates approach the
    boundary; a negative value means decreasing coordinates approach it.
    Motion away from the boundary is never reduced.
    """

    command = float(command_m_s)
    direction = 1.0 if float(toward_sign) >= 0.0 else -1.0
    toward_command = direction * command
    remaining = direction * (float(boundary_m) - float(position_m))
    margin = max(0.0, float(safety_margin_m))

    if toward_command <= 0.0:
        return command, max(0.0, remaining), max(0.0, float(max_speed_m_s))
    if remaining <= margin or measured_m_s is None:
        return 0.0, max(0.0, remaining), 0.0

    measured_toward = max(0.0, direction * float(measured_m_s))
    distance_after_delay = max(
        0.0,
        remaining
        - margin
        - measured_toward * max(0.0, float(reaction_time_s)),
    )
    decel = max(1.0e-6, float(braking_decel_m_s2))
    allowed = min(
        max(0.0, float(max_speed_m_s)),
        math.sqrt(2.0 * decel * distance_after_delay),
    )
    return direction * min(toward_command, allowed), remaining, allowed


def limit_bed_axis_velocity(
    vx_m_s: float,
    vy_m_s: float,
    measured_world_vx_m_s: Optional[float],
    measured_world_vy_m_s: Optional[float],
    x_m: float,
    y_m: float,
    *,
    target_x_m: float,
    target_y_m: float,
    side_toward_sign: float,
    max_speed_m_s: float = 3.0,
    forward_braking_decel_m_s2: float = 2.0,
    side_braking_decel_m_s2: float = 2.2,
    reaction_time_s: float = 0.22,
    safety_margin_m: float = 0.03,
) -> Tuple[float, float, dict]:
    """Apply independent map-X and bed-side map-Y braking envelopes."""

    capped_vx, capped_vy = scale_planar_velocity(
        vx_m_s, vy_m_s, max_speed_m_s
    )
    limited_vx, x_remaining, x_limit = limit_velocity_toward_boundary(
        capped_vx,
        measured_world_vx_m_s,
        x_m,
        target_x_m,
        1.0,
        braking_decel_m_s2=forward_braking_decel_m_s2,
        reaction_time_s=reaction_time_s,
        safety_margin_m=safety_margin_m,
        max_speed_m_s=max_speed_m_s,
    )
    limited_vy, side_remaining, side_limit = limit_velocity_toward_boundary(
        capped_vy,
        measured_world_vy_m_s,
        y_m,
        target_y_m,
        side_toward_sign,
        braking_decel_m_s2=side_braking_decel_m_s2,
        reaction_time_s=reaction_time_s,
        safety_margin_m=safety_margin_m,
        max_speed_m_s=max_speed_m_s,
    )
    return limited_vx, limited_vy, {
        "x_remaining_m": x_remaining,
        "x_limit_m_s": x_limit,
        "side_remaining_m": side_remaining,
        "side_limit_m_s": side_limit,
        "x_limited": abs(limited_vx - capped_vx) > 1.0e-9,
        "side_limited": abs(limited_vy - capped_vy) > 1.0e-9,
    }


def limit_bed_body_velocity(
    forward_m_s: float,
    left_m_s: float,
    measured_world_vx_m_s: Optional[float],
    measured_world_vy_m_s: Optional[float],
    yaw_rad: float,
    x_m: float,
    y_m: float,
    **options,
) -> Tuple[float, float, dict]:
    """Apply map-axis bed envelopes to a ROS base-frame command."""

    world_vx, world_vy = body_to_world_velocity(
        forward_m_s, left_m_s, yaw_rad
    )
    limited_world_vx, limited_world_vy, decision = limit_bed_axis_velocity(
        world_vx,
        world_vy,
        measured_world_vx_m_s,
        measured_world_vy_m_s,
        x_m,
        y_m,
        **options,
    )
    limited_forward, limited_left = world_to_body_velocity(
        limited_world_vx, limited_world_vy, yaw_rad
    )
    return limited_forward, limited_left, {
        **decision,
        "input_world_vx_m_s": world_vx,
        "input_world_vy_m_s": world_vy,
        "output_world_vx_m_s": limited_world_vx,
        "output_world_vy_m_s": limited_world_vy,
    }


def limit_home_velocity(
    vx_m_s: float,
    vy_m_s: float,
    x_m: float,
    y_m: float,
    *,
    home_x_m: float = 0.0,
    home_y_m: float = 0.0,
    max_speed_m_s: float = 3.0,
    soft_decel_m_s2: float = 1.425,
    terminal_speed_m_s: float = 0.10,
    terminal_distance_m: float = 0.10,
    decel_start_distance_m: Optional[float] = None,
) -> Tuple[float, float, float]:
    """Return the limited velocity and the computed distance/limit."""

    distance = distance_to_home(x_m, y_m, home_x_m, home_y_m)
    speed_limit = approach_speed_limit(
        distance,
        max_speed_m_s=max_speed_m_s,
        soft_decel_m_s2=soft_decel_m_s2,
        terminal_speed_m_s=terminal_speed_m_s,
        terminal_distance_m=terminal_distance_m,
        decel_start_distance_m=decel_start_distance_m,
    )
    limited_vx, limited_vy = scale_planar_velocity(vx_m_s, vy_m_s, speed_limit)
    return limited_vx, limited_vy, speed_limit


def home_approach_command(
    vx_m_s: float,
    vy_m_s: float,
    *,
    home_active: bool,
    pose_fresh: bool,
    x_m: float = 0.0,
    y_m: float = 0.0,
    home_x_m: float = 0.0,
    home_y_m: float = 0.0,
    max_speed_m_s: float = 3.0,
    soft_decel_m_s2: float = 1.425,
    terminal_speed_m_s: float = 0.10,
    terminal_distance_m: float = 0.10,
    decel_start_distance_m: Optional[float] = None,
) -> Tuple[float, float]:
    """Apply home gating without changing commands from other task states."""

    if not home_active:
        return float(vx_m_s), float(vy_m_s)
    if not pose_fresh:
        return 0.0, 0.0
    limited_vx, limited_vy, _ = limit_home_velocity(
        vx_m_s,
        vy_m_s,
        x_m,
        y_m,
        home_x_m=home_x_m,
        home_y_m=home_y_m,
        max_speed_m_s=max_speed_m_s,
        soft_decel_m_s2=soft_decel_m_s2,
        terminal_speed_m_s=terminal_speed_m_s,
        terminal_distance_m=terminal_distance_m,
        decel_start_distance_m=decel_start_distance_m,
    )
    return limited_vx, limited_vy
