"""Small, dependency-free 2-D scan matching helpers.

The guard deliberately fixes relative yaw to the HWT101CT value carried by
``/odom``.  ICP therefore estimates translation only and cannot pull heading
towards a person or another transient point cluster.
"""

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class Pose2D:
    stamp_ns: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MatchResult:
    translation: np.ndarray
    inlier_ratio: float
    rmse: float
    inlier_count: int
    geometry_ratio: float
    spatial_extent: float
    iterations: int


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_motion(previous: Pose2D, current: Pose2D) -> tuple[np.ndarray, float]:
    """Return the current-base to previous-base transform from two poses."""
    dx = current.x - previous.x
    dy = current.y - previous.y
    cosine = math.cos(previous.yaw)
    sine = math.sin(previous.yaw)
    translation = np.array(
        [cosine * dx + sine * dy, -sine * dx + cosine * dy],
        dtype=np.float64,
    )
    return translation, wrap_angle(current.yaw - previous.yaw)


def interpolate_pose(
    samples: Sequence[Pose2D], stamp_ns: int, max_error_s: float
) -> Pose2D | None:
    """Interpolate a timestamped pose without extrapolating stale odometry."""
    if not samples:
        return None
    if stamp_ns <= samples[0].stamp_ns:
        error_s = (samples[0].stamp_ns - stamp_ns) * 1e-9
        return samples[0] if error_s <= max_error_s else None
    if stamp_ns >= samples[-1].stamp_ns:
        error_s = (stamp_ns - samples[-1].stamp_ns) * 1e-9
        return samples[-1] if error_s <= max_error_s else None

    for left, right in zip(samples, samples[1:]):
        if left.stamp_ns <= stamp_ns <= right.stamp_ns:
            span = right.stamp_ns - left.stamp_ns
            if span <= 0:
                return left
            ratio = (stamp_ns - left.stamp_ns) / span
            yaw_delta = wrap_angle(right.yaw - left.yaw)
            return Pose2D(
                stamp_ns=stamp_ns,
                x=left.x + ratio * (right.x - left.x),
                y=left.y + ratio * (right.y - left.y),
                yaw=wrap_angle(left.yaw + ratio * yaw_delta),
            )
    return None


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Average XY points in deterministic square voxels."""
    points = np.asarray(points, dtype=np.float64)
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float64)
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((int(inverse.max()) + 1, 2), dtype=np.float64)
    counts = np.bincount(inverse)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


def transform_points(
    points: np.ndarray, translation: np.ndarray, yaw: float
) -> np.ndarray:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=np.float64)
    return np.asarray(points, dtype=np.float64) @ rotation.T + translation


def dilate_mask(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Dilate a boolean occupancy mask with a circular kernel."""
    source = np.asarray(mask, dtype=bool)
    if radius_cells <= 0:
        return source.copy()
    height, width = source.shape
    padded = np.pad(source, radius_cells, mode="constant")
    result = np.zeros_like(source)
    for row_offset in range(-radius_cells, radius_cells + 1):
        for column_offset in range(-radius_cells, radius_cells + 1):
            if row_offset ** 2 + column_offset ** 2 > radius_cells ** 2:
                continue
            row_start = radius_cells + row_offset
            column_start = radius_cells + column_offset
            result |= padded[
                row_start : row_start + height,
                column_start : column_start + width,
            ]
    return result


def points_in_grid_mask(
    points: np.ndarray,
    support_mask: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> np.ndarray:
    """Test world XY points against a row-major occupancy support mask."""
    points = np.asarray(points, dtype=np.float64)
    rows = np.floor((points[:, 1] - origin_y) / resolution).astype(np.int64)
    columns = np.floor((points[:, 0] - origin_x) / resolution).astype(np.int64)
    valid = (
        (rows >= 0)
        & (rows < support_mask.shape[0])
        & (columns >= 0)
        & (columns < support_mask.shape[1])
    )
    keep = np.zeros(len(points), dtype=bool)
    keep[valid] = support_mask[rows[valid], columns[valid]]
    return keep


def _nearest_neighbors(
    source: np.ndarray, target: np.ndarray, chunk_size: int = 128
) -> tuple[np.ndarray, np.ndarray]:
    indices = np.empty(len(source), dtype=np.int64)
    distances = np.empty(len(source), dtype=np.float64)
    for start in range(0, len(source), chunk_size):
        block = source[start : start + chunk_size]
        difference = block[:, None, :] - target[None, :, :]
        distance_sq = np.einsum("ijk,ijk->ij", difference, difference)
        nearest = np.argmin(distance_sq, axis=1)
        indices[start : start + len(block)] = nearest
        distances[start : start + len(block)] = np.sqrt(
            distance_sq[np.arange(len(block)), nearest]
        )
    return indices, distances


def _surface_normals(points: np.ndarray, neighbor_count: int = 8) -> np.ndarray:
    """Estimate an unoriented normal at every 2-D target point."""
    points = np.asarray(points, dtype=np.float64)
    count = min(max(3, neighbor_count), len(points))
    difference = points[:, None, :] - points[None, :, :]
    distance_sq = np.einsum("ijk,ijk->ij", difference, difference)
    neighbor_indices = np.argpartition(distance_sq, count - 1, axis=1)[:, :count]
    normals = np.empty_like(points)
    for index, neighbors in enumerate(neighbor_indices):
        local = points[neighbors]
        covariance = np.cov(local.T)
        _, eigenvectors = np.linalg.eigh(covariance)
        normals[index] = eigenvectors[:, 0]
    return normals


def _normal_geometry_ratio(normals: np.ndarray) -> float:
    information = np.asarray(normals, dtype=np.float64).T @ normals
    eigenvalues = np.linalg.eigvalsh(information)
    if eigenvalues[-1] <= 1e-12:
        return 0.0
    return float(max(0.0, eigenvalues[0] / eigenvalues[-1]))


def translation_icp(
    current_points: np.ndarray,
    previous_points: np.ndarray,
    relative_yaw: float,
    initial_translation: np.ndarray,
    max_correspondence_distance: float,
    max_iterations: int,
    trim_fraction: float = 0.75,
    convergence_tolerance: float = 0.001,
) -> MatchResult | None:
    """Match current scan into the previous scan with yaw held fixed.

    A trimmed point-to-line update lets non-parallel surfaces constrain both
    translation axes without being biased by changing Livox samples along a
    flat wall. Trimming, geometry/extent gates and consecutive windows keep a
    local moving cluster from becoming a trusted odometry source.
    """
    current = np.asarray(current_points, dtype=np.float64)
    previous = np.asarray(previous_points, dtype=np.float64)
    if len(current) < 3 or len(previous) < 3:
        return None

    translation = np.asarray(initial_translation, dtype=np.float64).copy()
    rotated = transform_points(current, np.zeros(2), relative_yaw)
    target_normals = _surface_normals(previous)
    used_iterations = 0

    for iteration in range(max_iterations):
        transformed = rotated + translation
        nearest, distances = _nearest_neighbors(transformed, previous)
        valid_indices = np.flatnonzero(distances <= max_correspondence_distance)
        if len(valid_indices) < 3:
            return None
        if trim_fraction < 1.0:
            retain = max(3, int(math.ceil(len(valid_indices) * trim_fraction)))
            order = np.argsort(distances[valid_indices])[:retain]
            valid_indices = valid_indices[order]
        matched_normals = target_normals[nearest[valid_indices]]
        residual = previous[nearest[valid_indices]] - transformed[valid_indices]
        projected_residual = np.einsum("ij,ij->i", matched_normals, residual)
        correction, _, _, _ = np.linalg.lstsq(
            matched_normals, projected_residual, rcond=None
        )
        translation += correction
        used_iterations = iteration + 1
        if float(np.linalg.norm(correction)) <= convergence_tolerance:
            break

    transformed = rotated + translation
    nearest, distances = _nearest_neighbors(transformed, previous)
    valid = distances <= max_correspondence_distance
    inlier_count = int(np.count_nonzero(valid))
    if inlier_count < 3:
        return None
    matched_targets = previous[nearest[valid]]
    matched_geometry = _normal_geometry_ratio(target_normals[nearest[valid]])
    span = np.ptp(matched_targets, axis=0)
    spatial_extent = float(np.hypot(span[0], span[1]))
    rmse = float(np.sqrt(np.mean(np.square(distances[valid]))))
    return MatchResult(
        translation=translation,
        inlier_ratio=inlier_count / float(len(current)),
        rmse=rmse,
        inlier_count=inlier_count,
        geometry_ratio=matched_geometry,
        spatial_extent=spatial_extent,
        iterations=used_iterations,
    )
