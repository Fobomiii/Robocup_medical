"""Pure geometry helpers for competition-cone footprint compensation."""

import math

import numpy as np


def future_timestamp_gap_s(
    requested_sec: int,
    requested_nanosec: int,
    latest_sec: int,
    latest_nanosec: int,
) -> float | None:
    """Return a non-negative future-extrapolation gap, otherwise ``None``."""
    requested_ns = int(requested_sec) * 1_000_000_000 + int(requested_nanosec)
    latest_ns = int(latest_sec) * 1_000_000_000 + int(latest_nanosec)
    gap_ns = requested_ns - latest_ns
    if gap_ns < 0:
        return None
    return gap_ns / 1_000_000_000.0


def merge_centers(
    centers: list[tuple[float, float]] | np.ndarray,
    merge_distance: float,
) -> list[tuple[float, float]]:
    """Merge nearby candidate centers into one physical-cone center.

    Clustering is transitive, so several fragments of the same cone cannot
    survive merely because the first and last fragments are slightly farther
    apart than ``merge_distance``.  The median keeps an outlying fragment from
    pulling the synthetic footprint toward a bed or wall.
    """
    if merge_distance <= 0.0:
        raise ValueError("merge_distance must be positive")
    if len(centers) == 0:
        return []

    points = np.asarray(centers, dtype=np.float64).reshape((-1, 2))
    unvisited = set(range(len(points)))
    groups: list[list[int]] = []
    while unvisited:
        seed = unvisited.pop()
        group = [seed]
        pending = [seed]
        while pending:
            current = pending.pop()
            neighbors = [
                index
                for index in unvisited
                if float(np.linalg.norm(points[current] - points[index]))
                <= merge_distance
            ]
            for index in neighbors:
                unvisited.remove(index)
                pending.append(index)
                group.append(index)
        groups.append(group)

    merged = []
    for group in groups:
        center = np.median(points[group], axis=0)
        merged.append((float(center[0]), float(center[1])))
    return merged


def compact_clusters(
    xyz: np.ndarray,
    min_z: float,
    max_z: float,
    min_range: float,
    max_range: float,
    cell_size: float,
    min_points: int,
    max_span: float,
    min_vertical_span: float,
    min_base_z: float = 0.0,
) -> list[np.ndarray]:
    """Return compact XY components that have measurable vertical extent.

    min_base_z drops clusters whose lowest return is above it.  A cone always
    has returns at ground level, so a cluster that starts higher up is
    something else -- most importantly the mounted arm, which sits at
    0.64-0.70 m and would otherwise be stamped as a 0.18 m disk at z=0.12 m,
    i.e. a phantom wall right at the robot's front edge.
    """
    if xyz.size == 0:
        return []

    radial_sq = xyz[:, 0] * xyz[:, 0] + xyz[:, 1] * xyz[:, 1]
    candidate_mask = (
        (xyz[:, 2] >= min_z)
        & (xyz[:, 2] <= max_z)
        & (radial_sq >= min_range * min_range)
        & (radial_sq <= max_range * max_range)
    )
    candidates = xyz[candidate_mask]
    if len(candidates) < min_points:
        return []

    cells: dict[tuple[int, int], list[int]] = {}
    for index, point in enumerate(candidates):
        key = (
            int(math.floor(float(point[0]) / cell_size)),
            int(math.floor(float(point[1]) / cell_size)),
        )
        cells.setdefault(key, []).append(index)

    clusters = []
    unvisited = set(cells)
    while unvisited:
        seed = unvisited.pop()
        pending = [seed]
        component = [seed]
        while pending:
            cell_x, cell_y = pending.pop()
            for offset_x in (-1, 0, 1):
                for offset_y in (-1, 0, 1):
                    neighbor = (cell_x + offset_x, cell_y + offset_y)
                    if neighbor in unvisited:
                        unvisited.remove(neighbor)
                        pending.append(neighbor)
                        component.append(neighbor)

        indices = [index for key in component for index in cells[key]]
        if len(indices) < min_points:
            continue
        points = candidates[indices]
        if min_base_z > 0.0 and float(np.min(points[:, 2])) > min_base_z:
            continue
        span_x = float(np.ptp(points[:, 0]))
        span_y = float(np.ptp(points[:, 1]))
        span_z = float(np.ptp(points[:, 2]))
        if max(span_x, span_y) > max_span or span_z < min_vertical_span:
            continue
        clusters.append(points)

    return clusters


def footprint_disk(
    center_x: float,
    center_y: float,
    radius: float,
    spacing: float,
    height: float,
) -> np.ndarray:
    """Create a filled horizontal disk for costmap and collision consumers."""
    offsets = np.arange(-radius, radius + spacing * 0.5, spacing)
    points = [
        (center_x + offset_x, center_y + offset_y, height)
        for offset_x in offsets
        for offset_y in offsets
        if offset_x * offset_x + offset_y * offset_y <= radius * radius
    ]
    points.extend(
        (
            center_x + radius * math.cos(angle),
            center_y + radius * math.sin(angle),
            height,
        )
        for angle in np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)
    )
    return np.asarray(points, dtype=np.float32)


def cone_axis_center(
    cluster: np.ndarray,
    physical_base_radius: float,
    cone_height: float,
) -> tuple[float, float]:
    """Estimate the cone axis from returns on its lidar-facing surface."""
    xy = cluster[:, :2].astype(np.float64, copy=False)
    ranges = np.linalg.norm(xy, axis=1)
    surface_radii = physical_base_radius * np.clip(
        1.0 - cluster[:, 2].astype(np.float64, copy=False) / cone_height,
        0.0,
        1.0,
    )
    candidates = xy.copy()
    valid = ranges > 1.0e-6
    candidates[valid] += (
        xy[valid] / ranges[valid, np.newaxis]
    ) * surface_radii[valid, np.newaxis]
    center = np.median(candidates, axis=0)
    return float(center[0]), float(center[1])


def expand_cone_footprints(
    xyz: np.ndarray,
    base_radius: float = 0.18,
    physical_base_radius: float = 0.155,
    cone_height: float = 0.65,
    min_z: float = 0.08,
    min_range: float = 0.30,
    max_range: float = 4.5,
    cluster_cell: float = 0.08,
    min_points: int = 5,
    max_span: float = 0.32,
    min_vertical_span: float = 0.12,
    center_merge_distance: float = 0.28,
    disk_spacing: float = 0.04,
    disk_height: float = 0.12,
    min_base_z: float = 0.35,
    return_diagnostics: bool = False,
):
    """Append known cone-base disks while preserving the filtered cloud."""
    clusters = compact_clusters(
        xyz,
        min_z,
        cone_height + 0.03,
        min_range,
        max_range,
        cluster_cell,
        min_points,
        max_span,
        min_vertical_span,
        min_base_z,
    )
    if not clusters:
        result = (np.ascontiguousarray(xyz, dtype=np.float32), [])
        return (*result, 0) if return_diagnostics else result

    raw_centers = []
    for cluster in clusters:
        center_x, center_y = cone_axis_center(
            cluster,
            physical_base_radius,
            cone_height,
        )
        raw_centers.append((center_x, center_y))

    centers = merge_centers(raw_centers, center_merge_distance)
    disks = []
    for center_x, center_y in centers:
        disks.append(
            footprint_disk(
                center_x,
                center_y,
                base_radius,
                disk_spacing,
                disk_height,
            )
        )

    expanded = np.vstack([xyz, *disks]).astype(np.float32, copy=False)
    result = (np.ascontiguousarray(expanded), centers)
    return (*result, len(raw_centers)) if return_diagnostics else result
