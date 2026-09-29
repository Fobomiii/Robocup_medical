"""Pure functions for the home-only approach speed envelope."""

from __future__ import annotations

import math
from typing import Tuple


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
    max_speed_m_s: float = 2.0,
    soft_decel_m_s2: float = 1.425,
    terminal_speed_m_s: float = 0.10,
    terminal_distance_m: float = 0.10,
) -> float:
    """Calculate a continuous speed limit for the remaining home distance.

    The limit is the braking envelope for a comfortable deceleration.  Inside
    the terminal radius it stays at ``terminal_speed_m_s`` so a task-state
    transition cannot turn a high-speed command into a hard stop.
    """

    max_speed = max(0.0, float(max_speed_m_s))
    soft_decel = max(0.0, float(soft_decel_m_s2))
    terminal_speed = max(0.0, float(terminal_speed_m_s))
    terminal_distance = max(0.0, float(terminal_distance_m))
    distance = max(0.0, float(distance_m))

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


def limit_home_velocity(
    vx_m_s: float,
    vy_m_s: float,
    x_m: float,
    y_m: float,
    *,
    home_x_m: float = 0.0,
    home_y_m: float = 0.0,
    max_speed_m_s: float = 2.0,
    soft_decel_m_s2: float = 1.425,
    terminal_speed_m_s: float = 0.10,
    terminal_distance_m: float = 0.10,
) -> Tuple[float, float, float]:
    """Return the limited velocity and the computed distance/limit."""

    distance = distance_to_home(x_m, y_m, home_x_m, home_y_m)
    speed_limit = approach_speed_limit(
        distance,
        max_speed_m_s=max_speed_m_s,
        soft_decel_m_s2=soft_decel_m_s2,
        terminal_speed_m_s=terminal_speed_m_s,
        terminal_distance_m=terminal_distance_m,
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
    max_speed_m_s: float = 2.0,
    soft_decel_m_s2: float = 1.425,
    terminal_speed_m_s: float = 0.10,
    terminal_distance_m: float = 0.10,
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
    )
    return limited_vx, limited_vy
