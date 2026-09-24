"""Pure geometry helpers for competition-cone footprint compensation."""

import math

import numpy as np


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
) -> list[np.ndarray]:
    """Return compact XY components that have measurable vertical extent."""
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


def expand_cone_footprints(
    xyz: np.ndarray,
    base_radius: float = 0.155,
    cone_height: float = 0.65,
    min_z: float = 0.08,
    min_range: float = 0.30,
    max_range: float = 4.5,
    cluster_cell: float = 0.08,
    min_points: int = 3,
    max_span: float = 0.32,
    min_vertical_span: float = 0.06,
    disk_spacing: float = 0.05,
    disk_height: float = 0.12,
) -> tuple[np.ndarray, list[tuple[float, float]]]:
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
    )
    if not clusters:
        return np.ascontiguousarray(xyz, dtype=np.float32), []

    disks = []
    centers = []
    for cluster in clusters:
        center_x = float(np.median(cluster[:, 0]))
        center_y = float(np.median(cluster[:, 1]))
        centers.append((center_x, center_y))
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
    return np.ascontiguousarray(expanded), centers
