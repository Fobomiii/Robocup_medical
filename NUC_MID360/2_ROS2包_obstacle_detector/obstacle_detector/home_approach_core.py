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


def clamp_axis_velocity(
    vx_m_s: float,
    vy_m_s: float,
    speed_limit_m_s: float,
) -> Tuple[float, float]:
    """Clamp X/Y independently for the cross-drive layout."""

    vx = float(vx_m_s)
    vy = float(vy_m_s)
    limit = max(0.0, float(speed_limit_m_s))
    return max(-limit, min(limit, vx)), max(-limit, min(limit, vy))


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
) -> Tuple[float, float, float, dict]:
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
    decel = max(1.0e-6, float(braking_decel_m_s2))
    reaction_time = max(0.0, float(reaction_time_s))
    measured_toward = (
        None
        if measured_m_s is None
        else max(0.0, direction * float(measured_m_s))
    )
    reaction_distance = (
        None if measured_toward is None else measured_toward * reaction_time
    )
    physical_braking_distance = (
        None
        if measured_toward is None
        else measured_toward * measured_toward / (2.0 * decel)
    )
    required_stop_distance = (
        None
        if measured_toward is None
        else margin + reaction_distance + physical_braking_distance
    )

    diagnostic = {
        "measured_toward_m_s": measured_toward,
        "reaction_time_s": reaction_time,
        "reaction_distance_m": reaction_distance,
        "braking_decel_m_s2": decel,
        "physical_braking_distance_m": physical_braking_distance,
        "safety_margin_m": margin,
        "required_stop_distance_m": required_stop_distance,
        "distance_after_reaction_m": None,
        "prebraking_active": False,
    }

    if toward_command <= 0.0:
        return (
            command,
            max(0.0, remaining),
            max(0.0, float(max_speed_m_s)),
            diagnostic,
        )
    if remaining <= margin or measured_m_s is None:
        diagnostic["prebraking_active"] = toward_command > 0.0
        return 0.0, max(0.0, remaining), 0.0, diagnostic

    distance_after_delay = max(
        0.0,
        remaining
        - margin
        - measured_toward * reaction_time,
    )
    allowed = min(
        max(0.0, float(max_speed_m_s)),
        math.sqrt(2.0 * decel * distance_after_delay),
    )
    diagnostic["distance_after_reaction_m"] = distance_after_delay
    diagnostic["prebraking_active"] = toward_command > allowed + 1.0e-9
    return (
        direction * min(toward_command, allowed),
        remaining,
        allowed,
        diagnostic,
    )


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

    capped_vx, capped_vy = clamp_axis_velocity(
        vx_m_s, vy_m_s, max_speed_m_s
    )
    limited_vx, x_remaining, x_limit, x_braking = limit_velocity_toward_boundary(
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
    limited_vy, side_remaining, side_limit, side_braking = limit_velocity_toward_boundary(
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
        "x_braking": x_braking,
        "side_braking": side_braking,
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


def limit_home_axis_velocity(
    vx_m_s: float,
    vy_m_s: float,
    measured_world_vx_m_s: Optional[float],
    measured_world_vy_m_s: Optional[float],
    x_m: float,
    y_m: float,
    *,
    target_x_m: float = 0.0,
    target_y_m: float = 0.0,
    max_speed_m_s: float = 3.0,
    braking_decel_m_s2: float = 2.0,
    reaction_time_s: float = 0.22,
    safety_margin_m: float = 0.03,
) -> Tuple[float, float, dict]:
    """Limit only the map axis that continues through the Home boundary.

    The competition Home boundary is behind the start reference, so motion
    toward decreasing map-X is the dangerous component in the ROS map frame.
    Map-Y remains available for lateral centring, and positive map-X remains
    available to leave the boundary after an overshoot or localisation jump.
    """

    capped_vx, capped_vy = clamp_axis_velocity(
        vx_m_s, vy_m_s, max_speed_m_s
    )
    limited_vx, remaining, speed_limit, braking = limit_velocity_toward_boundary(
        capped_vx,
        measured_world_vx_m_s,
        x_m,
        target_x_m,
        -1.0,
        braking_decel_m_s2=braking_decel_m_s2,
        reaction_time_s=reaction_time_s,
        safety_margin_m=safety_margin_m,
        max_speed_m_s=max_speed_m_s,
    )
    return limited_vx, capped_vy, {
        "boundary_x_m": float(target_x_m),
        "target_y_m": float(target_y_m),
        "remaining_m": remaining,
        "speed_limit_m_s": speed_limit,
        "boundary_limited": abs(limited_vx - capped_vx) > 1.0e-9,
        "lateral_preserved": abs(capped_vy) > 1.0e-9,
        "measured_world_vy_m_s": measured_world_vy_m_s,
        "braking": braking,
    }


def limit_home_body_velocity(
    forward_m_s: float,
    left_m_s: float,
    measured_world_vx_m_s: Optional[float],
    measured_world_vy_m_s: Optional[float],
    yaw_rad: float,
    x_m: float,
    y_m: float,
    **options,
) -> Tuple[float, float, dict]:
    """Apply the map-axis Home boundary envelope to a body command."""

    world_vx, world_vy = body_to_world_velocity(
        forward_m_s, left_m_s, yaw_rad
    )
    limited_world_vx, limited_world_vy, decision = limit_home_axis_velocity(
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


def limit_field_axis_velocity(
    vx_m_s: float,
    vy_m_s: float,
    measured_world_vx_m_s: Optional[float],
    measured_world_vy_m_s: Optional[float],
    x_m: float,
    y_m: float,
    *,
    field_x_min_m: float = -0.5,
    field_x_max_m: float = 6.5,
    field_y_min_m: float = -3.5,
    field_y_max_m: float = 3.5,
    footprint_radius_m: float = 0.24,
    localization_margin_m: float = 0.05,
    braking_decel_m_s2: float = 2.0,
    reaction_time_s: float = 0.22,
    braking_margin_m: float = 0.03,
    max_speed_m_s: float = 3.0,
) -> Tuple[float, float, dict]:
    """Keep the complete chassis footprint inside all four field edges.

    The surveyed limits describe the physical field.  The usable centre
    rectangle is inset by the footprint projection and localisation margin.
    Each map-axis component is limited independently, so braking toward one
    edge does not remove motion parallel to that edge or motion back inward.
    """

    radius = max(0.0, float(footprint_radius_m))
    pose_margin = max(0.0, float(localization_margin_m))
    inset = radius + pose_margin
    x_lower = float(field_x_min_m) + inset
    x_upper = float(field_x_max_m) - inset
    y_lower = float(field_y_min_m) + inset
    y_upper = float(field_y_max_m) - inset
    if x_lower >= x_upper or y_lower >= y_upper:
        raise ValueError("field boundary is smaller than the protected footprint")

    capped_vx, capped_vy = clamp_axis_velocity(vx_m_s, vy_m_s, max_speed_m_s)
    # A missing wheel sample must not stop the robot in the middle of the
    # field.  Use the capped command as the conservative delay-speed estimate;
    # the pose freshness gate remains responsible for stopping on stale pose.
    measured_vx = capped_vx if measured_world_vx_m_s is None else float(measured_world_vx_m_s)
    measured_vy = capped_vy if measured_world_vy_m_s is None else float(measured_world_vy_m_s)

    if capped_vx < 0.0:
        limited_vx, x_remaining, x_limit, x_braking = limit_velocity_toward_boundary(
            capped_vx, measured_vx, x_m, x_lower, -1.0,
            braking_decel_m_s2=braking_decel_m_s2,
            reaction_time_s=reaction_time_s, safety_margin_m=braking_margin_m,
            max_speed_m_s=max_speed_m_s,
        )
        x_edge = "x_min"
    elif capped_vx > 0.0:
        limited_vx, x_remaining, x_limit, x_braking = limit_velocity_toward_boundary(
            capped_vx, measured_vx, x_m, x_upper, 1.0,
            braking_decel_m_s2=braking_decel_m_s2,
            reaction_time_s=reaction_time_s, safety_margin_m=braking_margin_m,
            max_speed_m_s=max_speed_m_s,
        )
        x_edge = "x_max"
    else:
        limited_vx, x_remaining, x_limit, x_edge = 0.0, None, max_speed_m_s, None
        x_braking = None

    if capped_vy < 0.0:
        limited_vy, y_remaining, y_limit, y_braking = limit_velocity_toward_boundary(
            capped_vy, measured_vy, y_m, y_lower, -1.0,
            braking_decel_m_s2=braking_decel_m_s2,
            reaction_time_s=reaction_time_s, safety_margin_m=braking_margin_m,
            max_speed_m_s=max_speed_m_s,
        )
        y_edge = "y_min"
    elif capped_vy > 0.0:
        limited_vy, y_remaining, y_limit, y_braking = limit_velocity_toward_boundary(
            capped_vy, measured_vy, y_m, y_upper, 1.0,
            braking_decel_m_s2=braking_decel_m_s2,
            reaction_time_s=reaction_time_s, safety_margin_m=braking_margin_m,
            max_speed_m_s=max_speed_m_s,
        )
        y_edge = "y_max"
    else:
        limited_vy, y_remaining, y_limit, y_edge = 0.0, None, max_speed_m_s, None
        y_braking = None

    return limited_vx, limited_vy, {
        "usable_x_min_m": x_lower, "usable_x_max_m": x_upper,
        "usable_y_min_m": y_lower, "usable_y_max_m": y_upper,
        "x_edge": x_edge, "y_edge": y_edge,
        "x_remaining_m": x_remaining, "y_remaining_m": y_remaining,
        "x_limit_m_s": x_limit, "y_limit_m_s": y_limit,
        "x_limited": abs(limited_vx - capped_vx) > 1.0e-9,
        "y_limited": abs(limited_vy - capped_vy) > 1.0e-9,
        "x_braking": x_braking, "y_braking": y_braking,
        "wheel_fallback": measured_world_vx_m_s is None or measured_world_vy_m_s is None,
    }


def limit_field_body_velocity(
    forward_m_s: float,
    left_m_s: float,
    measured_world_vx_m_s: Optional[float],
    measured_world_vy_m_s: Optional[float],
    yaw_rad: float,
    x_m: float,
    y_m: float,
    **options,
) -> Tuple[float, float, dict]:
    """Apply the four-edge field guard to a ROS base-frame command."""

    world_vx, world_vy = body_to_world_velocity(forward_m_s, left_m_s, yaw_rad)
    limited_world_vx, limited_world_vy, decision = limit_field_axis_velocity(
        world_vx, world_vy, measured_world_vx_m_s, measured_world_vy_m_s,
        x_m, y_m, **options,
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
    limited_vx, limited_vy = clamp_axis_velocity(vx_m_s, vy_m_s, speed_limit)
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
