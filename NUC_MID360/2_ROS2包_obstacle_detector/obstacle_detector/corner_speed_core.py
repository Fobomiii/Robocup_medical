"""Geometry helpers for curvature-aware Nav2 speed limits."""

from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median
from typing import Optional, Sequence, Tuple


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class CornerSpeedDecision:
    speed_limit_m_s: float
    corner_distance_m: Optional[float]
    turn_angle_rad: float
    curvature_m_inv: float
    corner_speed_m_s: Optional[float] = None


@dataclass(frozen=True)
class ReferenceSpeedSample:
    distance_m: float
    x_m: float
    y_m: float
    curvature_m_inv: float
    radius_m: float
    curve_speed_m_s: float
    braking_speed_m_s: float
    reference_speed_m_s: float


@dataclass(frozen=True)
class ReferenceSpeedProfile:
    samples: tuple[ReferenceSpeedSample, ...]
    command_speed_m_s: float
    command_distance_m: float


@dataclass
class CornerLimitReleaseLatch:
    """Keep the corner ceiling latched until the exit is stably straight."""

    release_hold_s: float = 0.50
    straight_confirmation_s: float = 0.20
    armed: bool = False
    last_corner_s: Optional[float] = None
    straight_since_s: Optional[float] = None

    def __post_init__(self) -> None:
        if self.release_hold_s < 0.0:
            raise ValueError("corner release hold must be non-negative")
        if self.straight_confirmation_s < 0.0:
            raise ValueError("straight confirmation must be non-negative")

    def reset(self) -> None:
        self.armed = False
        self.last_corner_s = None
        self.straight_since_s = None

    def update(
        self,
        now_s: float,
        *,
        corner_detected: bool,
        straight_path_confirmed: bool,
    ) -> bool:
        """Return true while the post-corner speed ceiling must be held."""

        if not math.isfinite(now_s):
            raise ValueError("corner hold time must be finite")
        if corner_detected:
            self.armed = True
            self.last_corner_s = now_s
            self.straight_since_s = None
            return False
        if not self.armed or self.last_corner_s is None:
            return False

        if straight_path_confirmed:
            if self.straight_since_s is None:
                self.straight_since_s = now_s
        else:
            self.straight_since_s = None

        hold_elapsed = max(0.0, now_s - self.last_corner_s)
        straight_elapsed = (
            0.0
            if self.straight_since_s is None
            else max(0.0, now_s - self.straight_since_s)
        )
        if (
            hold_elapsed >= self.release_hold_s
            and straight_elapsed >= self.straight_confirmation_s
        ):
            self.reset()
            return False
        return True


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


def _resample_path(
    path_points: Sequence[Point2D],
    sample_step_m: float,
    maximum_distance_m: float,
) -> tuple[list[Point2D], list[float]]:
    cumulative = _cumulative_distances(path_points)
    path_length = min(cumulative[-1], maximum_distance_m)
    sample_distances = []
    distance_m = 0.0
    while distance_m < path_length - 1.0e-9:
        sample_distances.append(distance_m)
        distance_m += sample_step_m
    sample_distances.append(path_length)
    return (
        [
            _point_at_distance(path_points, cumulative, sample_distance_m)
            for sample_distance_m in sample_distances
        ],
        sample_distances,
    )


def _menger_curvature(first: Point2D, center: Point2D, last: Point2D) -> float:
    first_center = _distance(first, center)
    center_last = _distance(center, last)
    first_last = _distance(first, last)
    denominator = first_center * center_last * first_last
    if denominator <= 1.0e-12:
        return 0.0
    cross = (
        (center[0] - first[0]) * (last[1] - first[1])
        - (center[1] - first[1]) * (last[0] - first[0])
    )
    return 2.0 * abs(cross) / denominator


def _median_filter(values: Sequence[float], radius: int) -> list[float]:
    if radius <= 0:
        return list(values)
    return [
        float(median(values[max(0, index - radius) : index + radius + 1]))
        for index in range(len(values))
    ]


def reference_speed_profile(
    path_points: Sequence[Point2D],
    robot_x_m: float,
    robot_y_m: float,
    current_speed_m_s: float,
    *,
    max_speed_m_s: float = 3.0,
    min_curve_speed_m_s: float = 0.50,
    lateral_accel_m_s2: float = 2.30,
    accel_m_s2: float = 4.0,
    braking_decel_m_s2: float = 2.0,
    reaction_time_s: float = 0.20,
    braking_margin_m: float = 0.20,
    lookahead_distance_m: float = 6.0,
    sample_step_m: float = 0.10,
    curvature_span_m: float = 0.50,
    curvature_filter_window_m: float = 0.20,
    command_lookahead_m: float = 0.25,
) -> ReferenceSpeedProfile:
    """Build a diagnostic curvature speed profile without commanding the robot."""

    positive_values = (
        max_speed_m_s,
        min_curve_speed_m_s,
        lateral_accel_m_s2,
        accel_m_s2,
        braking_decel_m_s2,
        lookahead_distance_m,
        sample_step_m,
        curvature_span_m,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in positive_values):
        raise ValueError("reference speed parameters must be finite and positive")
    if min_curve_speed_m_s > max_speed_m_s:
        raise ValueError("minimum curve speed must not exceed maximum speed")
    if not math.isfinite(current_speed_m_s) or current_speed_m_s < 0.0:
        raise ValueError("current speed must be finite and non-negative")
    if reaction_time_s < 0.0 or braking_margin_m < 0.0:
        raise ValueError("reaction time and braking margin must be non-negative")
    if curvature_filter_window_m < 0.0 or command_lookahead_m < 0.0:
        raise ValueError("filter and command lookahead must be non-negative")

    forward_path = _path_ahead(path_points, robot_x_m, robot_y_m)
    if len(forward_path) < 2:
        return ReferenceSpeedProfile((), 0.0, 0.0)
    cumulative = _cumulative_distances(forward_path)
    if cumulative[-1] <= 1.0e-6:
        return ReferenceSpeedProfile((), 0.0, 0.0)

    sampled_points, distances = _resample_path(
        forward_path,
        sample_step_m,
        lookahead_distance_m,
    )
    path_length = distances[-1]
    raw_curvatures = []
    for distance_m in distances:
        before_distance_m = max(0.0, distance_m - curvature_span_m)
        after_distance_m = min(path_length, distance_m + curvature_span_m)
        if (
            distance_m - before_distance_m < sample_step_m * 0.5
            or after_distance_m - distance_m < sample_step_m * 0.5
        ):
            raw_curvatures.append(0.0)
            continue
        raw_curvatures.append(
            _menger_curvature(
                _point_at_distance(
                    forward_path, cumulative, before_distance_m
                ),
                _point_at_distance(forward_path, cumulative, distance_m),
                _point_at_distance(
                    forward_path, cumulative, after_distance_m
                ),
            )
        )

    filter_radius = int(
        round(0.5 * curvature_filter_window_m / sample_step_m)
    )
    curvatures = _median_filter(raw_curvatures, filter_radius)
    curve_speeds = [
        max_speed_m_s
        if curvature <= 1.0e-9
        else min(
            max_speed_m_s,
            max(min_curve_speed_m_s, math.sqrt(lateral_accel_m_s2 / curvature)),
        )
        for curvature in curvatures
    ]

    braking_speeds = list(curve_speeds)
    reaction_velocity = braking_decel_m_s2 * reaction_time_s
    for sample_index, sample_distance_m in enumerate(distances):
        allowed_speed_m_s = curve_speeds[sample_index]
        for target_index in range(sample_index + 1, len(distances)):
            target_speed_m_s = curve_speeds[target_index]
            if target_speed_m_s >= allowed_speed_m_s:
                continue
            braking_distance_m = max(
                0.0,
                distances[target_index] - sample_distance_m - braking_margin_m,
            )
            candidate_speed_m_s = (
                -reaction_velocity
                + math.sqrt(
                    reaction_velocity * reaction_velocity
                    + target_speed_m_s * target_speed_m_s
                    + 2.0 * braking_decel_m_s2 * braking_distance_m
                )
            )
            allowed_speed_m_s = min(allowed_speed_m_s, candidate_speed_m_s)
        braking_speeds[sample_index] = max(
            min_curve_speed_m_s,
            min(max_speed_m_s, allowed_speed_m_s),
        )

    bounded_current_speed_m_s = min(max_speed_m_s, current_speed_m_s)
    reference_speeds = [min(braking_speeds[0], bounded_current_speed_m_s)]
    for sample_index in range(1, len(distances)):
        segment_length_m = distances[sample_index] - distances[sample_index - 1]
        acceleration_ceiling_m_s = math.sqrt(
            reference_speeds[-1] * reference_speeds[-1]
            + 2.0 * accel_m_s2 * segment_length_m
        )
        reference_speeds.append(
            min(braking_speeds[sample_index], acceleration_ceiling_m_s)
        )

    samples = tuple(
        ReferenceSpeedSample(
            distance_m=distance_m,
            x_m=point[0],
            y_m=point[1],
            curvature_m_inv=curvature,
            radius_m=0.0 if curvature <= 1.0e-9 else 1.0 / curvature,
            curve_speed_m_s=curve_speed_m_s,
            braking_speed_m_s=braking_speed_m_s,
            reference_speed_m_s=reference_speed_m_s,
        )
        for (
            distance_m,
            point,
            curvature,
            curve_speed_m_s,
            braking_speed_m_s,
            reference_speed_m_s,
        ) in zip(
            distances,
            sampled_points,
            curvatures,
            curve_speeds,
            braking_speeds,
            reference_speeds,
        )
    )
    command_sample = min(
        samples,
        key=lambda sample: abs(sample.distance_m - command_lookahead_m),
    )
    return ReferenceSpeedProfile(
        samples,
        command_sample.reference_speed_m_s,
        command_sample.distance_m,
    )


def corner_speed_limit(
    path_points: Sequence[Point2D],
    robot_x_m: float,
    robot_y_m: float,
    *,
    max_speed_m_s: float = 3.0,
    min_corner_speed_m_s: float = 0.50,
    max_corner_speed_m_s: float = 1.00,
    lateral_accel_m_s2: float = 2.30,
    braking_decel_m_s2: float = 2.30,
    lookahead_distance_m: float = 3.50,
    tangent_span_m: float = 0.35,
    sample_step_m: float = 0.10,
    min_turn_angle_rad: float = math.radians(25.0),
    braking_margin_m: float = 0.05,
    reaction_time_s: float = 0.20,
) -> CornerSpeedDecision:
    """Return a braking-envelope speed limit for the sharpest corner ahead."""

    positive_values = (
        max_speed_m_s,
        min_corner_speed_m_s,
        max_corner_speed_m_s,
        lateral_accel_m_s2,
        braking_decel_m_s2,
        lookahead_distance_m,
        tangent_span_m,
        sample_step_m,
    )
    if any(value <= 0.0 for value in positive_values):
        raise ValueError("corner speed parameters must be positive")
    if min_corner_speed_m_s > max_corner_speed_m_s:
        raise ValueError("minimum corner speed must not exceed its maximum")
    if max_corner_speed_m_s > max_speed_m_s:
        raise ValueError("maximum corner speed must not exceed route speed")
    if reaction_time_s < 0.0:
        raise ValueError("corner speed reaction time must be non-negative")

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
                    max_corner_speed_m_s,
                    max(min_corner_speed_m_s, corner_speed),
                )
                braking_distance = max(0.0, center_distance - braking_margin_m)
                reaction_velocity = braking_decel_m_s2 * reaction_time_s
                allowed_speed = (
                    -reaction_velocity
                    + math.sqrt(
                        reaction_velocity * reaction_velocity
                        + corner_speed * corner_speed
                        + 2.0 * braking_decel_m_s2 * braking_distance
                    )
                )
                allowed_speed = min(
                    max_speed_m_s,
                    max(corner_speed, allowed_speed),
                )
                if allowed_speed < best_decision.speed_limit_m_s:
                    best_decision = CornerSpeedDecision(
                        allowed_speed,
                        center_distance,
                        turn_angle,
                        curvature,
                        corner_speed,
                    )
        center_distance += sample_step_m

    return best_decision
