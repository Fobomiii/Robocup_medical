"""Geometry helpers for curvature-aware Nav2 speed limits."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class CornerSpeedDecision:
    speed_limit_m_s: float
    corner_distance_m: Optional[float]
    turn_angle_rad: float
    curvature_m_inv: float


def _distance(first: Point2D, second: Point2D) -> float:
    return math.hypot(second[0] - first[0], second[1] - first[1])


def _path_ahead(
    path_points: Sequence[Point2D], robot_x_m: float, robot_y_m: float
) -> list[Point2D]:
    if len(path_points) < 2:
        return []

    best_distance_sq = math.inf
    best_segment_index = 0
    best_projection = path_points[0]
    for segment_index in range(len(path_points) - 1):
        start_x, start_y = path_points[segment_index]
        end_x, end_y = path_points[segment_index + 1]
        segment_x = end_x - start_x
        segment_y = end_y - start_y
        segment_length_sq = segment_x * segment_x + segment_y * segment_y
        if segment_length_sq <= 1.0e-12:
            projection_ratio = 0.0
        else:
            projection_ratio = (
                (robot_x_m - start_x) * segment_x
                + (robot_y_m - start_y) * segment_y
            ) / segment_length_sq
            projection_ratio = min(1.0, max(0.0, projection_ratio))
        projection = (
            start_x + projection_ratio * segment_x,
            start_y + projection_ratio * segment_y,
        )
        distance_sq = (
            (robot_x_m - projection[0]) ** 2
            + (robot_y_m - projection[1]) ** 2
        )
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_segment_index = segment_index
            best_projection = projection

    remaining_path = [best_projection]
    for point in path_points[best_segment_index + 1 :]:
        if _distance(remaining_path[-1], point) > 1.0e-6:
            remaining_path.append(point)
    return remaining_path


def _cumulative_distances(path_points: Sequence[Point2D]) -> list[float]:
    distances = [0.0]
    for point_index in range(1, len(path_points)):
        distances.append(
            distances[-1] + _distance(path_points[point_index - 1], path_points[point_index])
        )
    return distances


def _point_at_distance(
    path_points: Sequence[Point2D], cumulative: Sequence[float], distance_m: float
) -> Point2D:
    bounded_distance = min(max(0.0, distance_m), cumulative[-1])
    for point_index in range(1, len(cumulative)):
        if cumulative[point_index] < bounded_distance:
            continue
        segment_length = cumulative[point_index] - cumulative[point_index - 1]
        if segment_length <= 1.0e-12:
            return path_points[point_index]
        ratio = (bounded_distance - cumulative[point_index - 1]) / segment_length
        start_x, start_y = path_points[point_index - 1]
        end_x, end_y = path_points[point_index]
        return (
            start_x + ratio * (end_x - start_x),
            start_y + ratio * (end_y - start_y),
        )
    return path_points[-1]


def _heading(first: Point2D, second: Point2D) -> float:
    return math.atan2(second[1] - first[1], second[0] - first[0])


def _angle_difference(first_rad: float, second_rad: float) -> float:
    return abs(
        math.atan2(
            math.sin(second_rad - first_rad),
            math.cos(second_rad - first_rad),
        )
    )


def corner_speed_limit(
    path_points: Sequence[Point2D],
    robot_x_m: float,
    robot_y_m: float,
    *,
    max_speed_m_s: float = 2.0,
    min_corner_speed_m_s: float = 1.0,
    lateral_accel_m_s2: float = 1.40,
    braking_decel_m_s2: float = 1.30,
    lookahead_distance_m: float = 1.50,
    tangent_span_m: float = 0.35,
    sample_step_m: float = 0.10,
    min_turn_angle_rad: float = math.radians(25.0),
    braking_margin_m: float = 0.05,
) -> CornerSpeedDecision:
    """Return a braking-envelope speed limit for the sharpest corner ahead."""

    positive_values = (
        max_speed_m_s,
        min_corner_speed_m_s,
        lateral_accel_m_s2,
        braking_decel_m_s2,
        lookahead_distance_m,
        tangent_span_m,
        sample_step_m,
    )
    if any(value <= 0.0 for value in positive_values):
        raise ValueError("corner speed parameters must be positive")

    forward_path = _path_ahead(path_points, robot_x_m, robot_y_m)
    if len(forward_path) < 3:
        return CornerSpeedDecision(max_speed_m_s, None, 0.0, 0.0)

    cumulative = _cumulative_distances(forward_path)
    path_length = cumulative[-1]
    maximum_distance = min(lookahead_distance_m, path_length - sample_step_m)
    if maximum_distance <= sample_step_m:
        return CornerSpeedDecision(max_speed_m_s, None, 0.0, 0.0)

    best_decision = CornerSpeedDecision(max_speed_m_s, None, 0.0, 0.0)
    center_distance = sample_step_m
    while center_distance <= maximum_distance + 1.0e-9:
        before_distance = max(0.0, center_distance - tangent_span_m)
        after_distance = min(path_length, center_distance + tangent_span_m)
        before_length = center_distance - before_distance
        after_length = after_distance - center_distance
        if before_length >= sample_step_m * 0.5 and after_length >= sample_step_m * 0.5:
            before_point = _point_at_distance(
                forward_path, cumulative, before_distance
            )
            center_point = _point_at_distance(
                forward_path, cumulative, center_distance
            )
            after_point = _point_at_distance(forward_path, cumulative, after_distance)
            incoming_heading = _heading(before_point, center_point)
            outgoing_heading = _heading(center_point, after_point)
            turn_angle = _angle_difference(incoming_heading, outgoing_heading)
            if turn_angle >= min_turn_angle_rad:
                tangent_midpoint_separation = 0.5 * (before_length + after_length)
                curvature = turn_angle / max(tangent_midpoint_separation, 1.0e-6)
                corner_speed = math.sqrt(lateral_accel_m_s2 / curvature)
                corner_speed = min(
                    max_speed_m_s,
                    max(min_corner_speed_m_s, corner_speed),
                )
                braking_distance = max(0.0, center_distance - braking_margin_m)
                allowed_speed = math.sqrt(
                    corner_speed * corner_speed
                    + 2.0 * braking_decel_m_s2 * braking_distance
                )
                allowed_speed = min(max_speed_m_s, allowed_speed)
                if allowed_speed < best_decision.speed_limit_m_s:
                    best_decision = CornerSpeedDecision(
                        allowed_speed,
                        center_distance,
                        turn_angle,
                        curvature,
                    )
        center_distance += sample_step_m

    return best_decision
