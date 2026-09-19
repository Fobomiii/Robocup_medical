"""Obstacle mapping and grid planning without ROS dependencies."""

from dataclasses import dataclass
import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .fixed_routes import Goal, Waypoint


@dataclass(frozen=True)
class Obstacle:
    x_mm: float
    y_mm: float
    radius_mm: float
    hits: int = 0


class ObstacleMapper:
    """Grid-based occupancy map for stable obstacle positions."""

    grid_resolution_mm = 100.0

    def __init__(
        self,
        field: dict,
        fixtures: Sequence[dict],
        *,
        range_min_m: float = 0.25,
        range_max_m: float = 6.5,
        z_min_m: float = -0.10,
        z_max_m: float = 0.55,
        lidar_forward_mm: float = 0.0,
        lidar_right_mm: float = 0.0,
        lidar_inverted: bool = False,
        fixture_reject_margin_mm: float = 120.0,
        field_reject_margin_mm: float = 80.0,
        cluster_cell_mm: float = 80.0,
        cluster_tolerance_mm: float = 180.0,
        cluster_min_points: int = 5,
        cluster_max_span_mm: float = 650.0,
        obstacle_radius_min_mm: float = 120.0,
        obstacle_radius_max_mm: float = 350.0,
        track_match_mm: float = 350.0,
        confirm_hits: int = 3,
        candidate_ttl_s: float = 1.0,
        confirmed_ttl_s: float = 30.0,
        max_obstacles: int = 5,
        map_change_mm: float = 80.0,
    ) -> None:
        self.field = field
        self.fixtures = fixtures
        self.range_min_m = range_min_m
        self.range_max_m = range_max_m
        self.z_min_m = z_min_m
        self.z_max_m = z_max_m
        self.lidar_forward_mm = lidar_forward_mm
        self.lidar_right_mm = lidar_right_mm
        self.lidar_inverted = lidar_inverted
        self.fixture_reject_margin_mm = fixture_reject_margin_mm
        self.field_reject_margin_mm = field_reject_margin_mm
        self.obstacle_radius_min_mm = obstacle_radius_min_mm
        self.obstacle_radius_max_mm = obstacle_radius_max_mm
        self.confirm_hits = confirm_hits
        self.candidate_ttl_s = candidate_ttl_s
        self.confirmed_ttl_s = confirmed_ttl_s
        self.max_obstacles = max_obstacles
        self.revision = 0
        # Parallel to the list from obstacles(); rebuilt on each call.
        self._blob_dynamic: List[bool] = []

        # Occupancy grid aligned to planner resolution
        self.grid_x_min = float(field["x_min_mm"])
        self.grid_y_min = float(field["y_min_mm"])
        x_max = float(field["x_max_mm"])
        y_max = float(field["y_max_mm"])
        self.grid_w = int(round((x_max - self.grid_x_min) / self.grid_resolution_mm))
        self.grid_h = int(round((y_max - self.grid_y_min) / self.grid_resolution_mm))
        self.grid_hits = np.zeros((self.grid_h, self.grid_w), dtype=np.int16)
        self.grid_last_seen = np.zeros((self.grid_h, self.grid_w), dtype=np.float64)
        # Cells newly occupied by an object that just vacated a neighbouring
        # cell. Rebuilt every update; used only to colour RViz markers.
        self.grid_dynamic = np.zeros((self.grid_h, self.grid_w), dtype=bool)

        # Precompute fixture mask for costmap visualisation
        self.fixture_mask = np.zeros((self.grid_h, self.grid_w), dtype=bool)
        for iy in range(self.grid_h):
            cy = self.grid_y_min + (iy + 0.5) * self.grid_resolution_mm
            for ix in range(self.grid_w):
                cx = self.grid_x_min + (ix + 0.5) * self.grid_resolution_mm
                for fix in fixtures:
                    hw = float(fix["width_mm"]) * 0.5
                    hh = float(fix["height_mm"]) * 0.5
                    if (abs(cx - float(fix["x_mm"])) <= hw
                            and abs(cy - float(fix["y_mm"])) <= hh):
                        self.fixture_mask[iy, ix] = True
                        break

    def update(
        self,
        sensor_points_m: np.ndarray,
        robot_x_mm: float,
        robot_y_mm: float,
        robot_yaw_deg: float,
        now_s: float,
    ) -> bool:
        world_pts = self._detect(
            sensor_points_m, robot_x_mm, robot_y_mm, robot_yaw_deg
        )
        before = self._confirmed_signature()
        self.grid_dynamic[:] = False

        # Mark cells seen this frame (each cell gets at most +1 per frame)

        seen = set()
        if len(world_pts) > 0:
            ix = np.floor(
                (world_pts[:, 0] - self.grid_x_min) / self.grid_resolution_mm
            ).astype(np.int32)
            iy = np.floor(
                (world_pts[:, 1] - self.grid_y_min) / self.grid_resolution_mm
            ).astype(np.int32)
            valid = (ix >= 0) & (ix < self.grid_w) & (iy >= 0) & (iy < self.grid_h)
            for idx in np.where(valid)[0]:
                seen.add((int(iy[idx]), int(ix[idx])))
        for ri, ci in seen:
            self.grid_hits[ri, ci] = min(int(self.grid_hits[ri, ci]) + 1, 255)
            self.grid_last_seen[ri, ci] = now_s

        # Annotate (never remove) cells whose object appears to have moved: a
        # confirmed cell gone stale while a confirmed neighbour is fresh. This
        # only colours RViz markers; the cell stays locked until its TTL.
        if seen:
            confirmed_mask = self.grid_hits >= self.confirm_hits
            stale = confirmed_mask & ((now_s - self.grid_last_seen) > 1.0)
            fresh = confirmed_mask & ((now_s - self.grid_last_seen) <= 0.5)
            for ri, ci in np.argwhere(stale):
                ri, ci = int(ri), int(ci)
                y0, y1 = max(0, ri - 1), min(self.grid_h, ri + 2)
                x0, x1 = max(0, ci - 1), min(self.grid_w, ci + 2)
                if fresh[y0:y1, x0:x1].any():
                    self.grid_dynamic[ri, ci] = True

        # Expire stale cells
        active = self.grid_hits > 0
        if active.any():
            age = now_s - self.grid_last_seen
            confirmed = self.grid_hits >= self.confirm_hits
            stale = (confirmed & (age > self.confirmed_ttl_s)) | (
                active & ~confirmed & (age > self.candidate_ttl_s)
            )
            self.grid_hits[stale] = 0
            self.grid_last_seen[stale] = 0.0

        after = self._confirmed_signature()
        if before != after:
            self.revision += 1
            return True
        return False

    def obstacles(self) -> List[Obstacle]:
        self._blob_dynamic: List[bool] = []
        confirmed = self.grid_hits >= self.confirm_hits
        if not confirmed.any():
            return []
        # Merge blobs that are within one cell of each other: sparse returns
        # leave the far side of one object as disconnected cells, which would
        # otherwise be reported as several separate obstacles.
        seeds = self._dilate(confirmed)
        blobs = self._find_blobs(seeds)
        ranked: List[Tuple[Obstacle, bool]] = []
        for blob in blobs:
            cells = [(ri, ci) for ri, ci in blob if confirmed[ri, ci]]
            if not cells:
                continue
            xs = [self.grid_x_min + (ci + 0.5) * self.grid_resolution_mm for _, ci in cells]
            ys = [self.grid_y_min + (ri + 0.5) * self.grid_resolution_mm for ri, _ in cells]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            if len(cells) == 1:
                radius = self.obstacle_radius_min_mm
            else:
                span_x = max(xs) - min(xs) + self.grid_resolution_mm
                span_y = max(ys) - min(ys) + self.grid_resolution_mm
                radius = 0.5 * math.hypot(span_x, span_y)
                radius = max(self.obstacle_radius_min_mm,
                             min(self.obstacle_radius_max_mm, radius))
            total_hits = sum(int(self.grid_hits[ri, ci]) for ri, ci in cells)
            moved = any(self.grid_dynamic[ri, ci] for ri, ci in cells)
            ranked.append((Obstacle(cx, cy, radius, total_hits), moved))
        ranked.sort(key=lambda item: -item[0].hits)
        ranked = ranked[: self.max_obstacles]
        self._blob_dynamic = [moved for _, moved in ranked]
        return [obstacle for obstacle, _ in ranked]

    @staticmethod
    def _dilate(mask: np.ndarray) -> np.ndarray:
        out = mask.copy()
        out[1:, :] |= mask[:-1, :]
        out[:-1, :] |= mask[1:, :]
        out[:, 1:] |= mask[:, :-1]
        out[:, :-1] |= mask[:, 1:]
        out[1:, 1:] |= mask[:-1, :-1]
        out[:-1, :-1] |= mask[1:, 1:]
        out[1:, :-1] |= mask[:-1, 1:]
        out[:-1, 1:] |= mask[1:, :-1]
        return out

    def obstacle_is_dynamic(self, index: int) -> bool:
        """True when the obstacle at ``index`` was produced by a moving object.

        ``index`` is the position in the list returned by ``obstacles()``, which
        is sorted by hit count. Call ``obstacles()`` first: the flags are
        rebuilt on each call and are not retained afterwards.
        """
        if not self._blob_dynamic:
            return False
        if index < 0 or index >= len(self._blob_dynamic):
            return False
        return bool(self._blob_dynamic[index])

    def costmap_data(self) -> np.ndarray:
        """Occupancy values: 0=free, 50=candidate, 100=confirmed, -1=fixture."""
        data = np.zeros((self.grid_h, self.grid_w), dtype=np.int8)
        data[(self.grid_hits > 0) & (self.grid_hits < self.confirm_hits)] = 50
        data[self.grid_hits >= self.confirm_hits] = 100
        data[self.fixture_mask] = -1
        return data

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _confirmed_signature(self) -> frozenset:
        return frozenset(map(tuple, np.argwhere(self.grid_hits >= self.confirm_hits)))

    def _detect(
        self,
        points: np.ndarray,
        robot_x_mm: float,
        robot_y_mm: float,
        robot_yaw_deg: float,
    ) -> np.ndarray:
        """Return world-frame (x_mm, y_mm) points after filtering."""
        if len(points) == 0:
            return np.empty((0, 2), dtype=np.float64)

        xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if self.lidar_inverted:
            xyz[:, 2] = -xyz[:, 2]
        finite = np.isfinite(xyz).all(axis=1)
        distance = np.hypot(xyz[:, 0], xyz[:, 1])
        mask = (
            finite
            & (distance >= self.range_min_m)
            & (distance <= self.range_max_m)
            & (xyz[:, 2] >= self.z_min_m)
            & (xyz[:, 2] <= self.z_max_m)
        )
        xyz = xyz[mask]
        if len(xyz) == 0:
            return np.empty((0, 2), dtype=np.float64)

        forward_mm = xyz[:, 0].astype(np.float64) * 1000.0 + self.lidar_forward_mm
        right_mm = -xyz[:, 1].astype(np.float64) * 1000.0 + self.lidar_right_mm
        yaw = math.radians(robot_yaw_deg)
        world_x = robot_x_mm + right_mm * math.cos(yaw) + forward_mm * math.sin(yaw)
        world_y = robot_y_mm - right_mm * math.sin(yaw) + forward_mm * math.cos(yaw)

        keep = self._field_mask(world_x, world_y)
        for fixture in self.fixtures:
            half_width = float(fixture["width_mm"]) * 0.5 + self.fixture_reject_margin_mm
            half_height = float(fixture["height_mm"]) * 0.5 + self.fixture_reject_margin_mm
            inside = (
                (np.abs(world_x - float(fixture["x_mm"])) <= half_width)
                & (np.abs(world_y - float(fixture["y_mm"])) <= half_height)
            )
            keep &= ~inside

        return np.column_stack((world_x[keep], world_y[keep]))

    def _field_mask(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        margin = self.field_reject_margin_mm
        return (
            (x >= float(self.field["x_min_mm"]) + margin)
            & (x <= float(self.field["x_max_mm"]) - margin)
            & (y >= float(self.field["y_min_mm"]) + margin)
            & (y <= float(self.field["y_max_mm"]) - margin)
        )

    @staticmethod
    def _find_blobs(mask: np.ndarray) -> List[List[Tuple[int, int]]]:
        h, w = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        blobs: List[List[Tuple[int, int]]] = []
        for iy in range(h):
            for ix in range(w):
                if not mask[iy, ix] or visited[iy, ix]:
                    continue
                blob: List[Tuple[int, int]] = []
                stack = [(iy, ix)]
                while stack:
                    cy, cx = stack.pop()
                    if visited[cy, cx]:
                        continue
                    visited[cy, cx] = True
                    blob.append((cy, cx))
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            if dy == 0 and dx == 0:
                                continue
                            ny, nx = cy + dy, cx + dx
                            if 0 <= ny < h and 0 <= nx < w:
                                if mask[ny, nx] and not visited[ny, nx]:
                                    stack.append((ny, nx))
                blobs.append(blob)
        return blobs


class GridPlanner:
    """Eight-connected A* with continuous collision checks and LOS smoothing."""

    _NEIGHBORS = (
        (-1, -1, math.sqrt(2.0)),
        (-1, 0, 1.0),
        (-1, 1, math.sqrt(2.0)),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (1, -1, math.sqrt(2.0)),
        (1, 0, 1.0),
        (1, 1, math.sqrt(2.0)),
    )

    def __init__(
        self,
        field: dict,
        fixtures: Sequence[dict],
        *,
        resolution_mm: float = 100.0,
        inflation_mm: float = 360.0,
        cruise_speed_mm_s: float = 120.0,
        corner_speed_mm_s: float = 90.0,
        max_waypoints: int = 16,
    ) -> None:
        self.field = field
        self.fixtures = fixtures
        self.resolution_mm = resolution_mm
        self.inflation_mm = inflation_mm
        self.cruise_speed_mm_s = cruise_speed_mm_s
        self.corner_speed_mm_s = corner_speed_mm_s
        self.max_waypoints = max_waypoints
        self.x_min = float(field["x_min_mm"])
        self.x_max = float(field["x_max_mm"])
        self.y_min = float(field["y_min_mm"])
        self.y_max = float(field["y_max_mm"])
        self.width = int(math.floor((self.x_max - self.x_min) / resolution_mm)) + 1
        self.height = int(math.floor((self.y_max - self.y_min) / resolution_mm)) + 1

    def plan(
        self,
        start_x_mm: float,
        start_y_mm: float,
        goal: Goal,
        obstacles: Sequence[Obstacle],
    ) -> Optional[List[Waypoint]]:
        blocked = self._blocked_cells(obstacles)
        start = self._nearest_free(self._to_cell(start_x_mm, start_y_mm), blocked)
        target = self._nearest_free(self._to_cell(goal.x_mm, goal.y_mm), blocked)
        if start is None or target is None:
            return None

        cells = self._astar(start, target, blocked)
        if not cells:
            return None

        coordinates = [(start_x_mm, start_y_mm)]
        coordinates.extend(self._to_world(cell) for cell in cells[1:-1])
        coordinates.append((goal.x_mm, goal.y_mm))
        coordinates = self._smooth(coordinates, obstacles)
        if len(coordinates) < 2:
            return None

        # Prefer the densest spacing that still fits the link's waypoint budget.
        # A longer route needs coarser spacing, not an abandoned plan: returning
        # None here would silently drop back to the fixed route.
        waypoints: Optional[List[Waypoint]] = None
        for max_seg_mm in (400.0, 600.0, 900.0, 1500.0):
            candidate = self._densify(coordinates, max_seg_mm)
            if len(candidate) > self.max_waypoints:
                continue
            built = self._build_waypoints(candidate, goal)
            if built is not None:
                waypoints = built
                break
        if waypoints is None:
            waypoints = self._build_waypoints(coordinates, goal)
        return waypoints

    def _build_waypoints(
        self,
        coordinates: Sequence[Tuple[float, float]],
        goal: Goal,
    ) -> Optional[List[Waypoint]]:
        waypoints: List[Waypoint] = []
        targets = coordinates[1:]
        for index, (x_mm, y_mm) in enumerate(targets):
            prev_x, prev_y = coordinates[index]
            travel_yaw = math.degrees(math.atan2(x_mm - prev_x, y_mm - prev_y))
            if index == len(targets) - 1:
                speed = self.corner_speed_mm_s
                waypoints.append(Waypoint(x_mm, y_mm, travel_yaw, speed))
                if abs(self._angle_diff(goal.yaw_deg, travel_yaw)) > 5.0:
                    waypoints.append(Waypoint(x_mm, y_mm, goal.yaw_deg, 0.0))
            else:
                speed = self.corner_speed_mm_s if index == 0 else self.cruise_speed_mm_s
                waypoints.append(Waypoint(x_mm, y_mm, travel_yaw, speed))
        if len(waypoints) > self.max_waypoints:
            return None
        return waypoints

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        d = a - b
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        return d

    def path_blocked(
        self,
        start: Tuple[float, float],
        waypoints: Sequence[Waypoint],
        obstacles: Sequence[Obstacle],
    ) -> bool:
        points = [start] + [(point.x_mm, point.y_mm) for point in waypoints]
        for first, second in zip(points, points[1:]):
            for obstacle in obstacles:
                clearance = obstacle.radius_mm + self.inflation_mm
                if self._point_segment_distance(obstacle.x_mm, obstacle.y_mm, first, second) <= clearance:
                    return True
        return False

    def point_is_free(
        self, x_mm: float, y_mm: float, obstacles: Sequence[Obstacle]
    ) -> bool:
        margin = self.inflation_mm
        if not (
            self.x_min + margin <= x_mm <= self.x_max - margin
            and self.y_min + margin <= y_mm <= self.y_max - margin
        ):
            return False
        for fixture in self.fixtures:
            half_width = float(fixture["width_mm"]) * 0.5 + margin
            half_height = float(fixture["height_mm"]) * 0.5 + margin
            if (
                abs(x_mm - float(fixture["x_mm"])) <= half_width
                and abs(y_mm - float(fixture["y_mm"])) <= half_height
            ):
                return False
        for obstacle in obstacles:
            clearance = obstacle.radius_mm + margin
            if math.hypot(x_mm - obstacle.x_mm, y_mm - obstacle.y_mm) <= clearance:
                return False
        return True

    def segment_is_free(
        self,
        first: Tuple[float, float],
        second: Tuple[float, float],
        obstacles: Sequence[Obstacle],
    ) -> bool:
        distance = math.hypot(second[0] - first[0], second[1] - first[1])
        samples = max(1, int(math.ceil(distance / (self.resolution_mm * 0.4))))
        for step in range(samples + 1):
            ratio = step / samples
            x_mm = first[0] + (second[0] - first[0]) * ratio
            y_mm = first[1] + (second[1] - first[1]) * ratio
            if not self.point_is_free(x_mm, y_mm, obstacles):
                return False
        return True

    def _blocked_cells(self, obstacles: Sequence[Obstacle]) -> set:
        blocked = set()
        for ix in range(self.width):
            for iy in range(self.height):
                x_mm, y_mm = self._to_world((ix, iy))
                if not self.point_is_free(x_mm, y_mm, obstacles):
                    blocked.add((ix, iy))
        return blocked

    def _nearest_free(self, origin: Tuple[int, int], blocked: set) -> Optional[Tuple[int, int]]:
        if self._inside(origin) and origin not in blocked:
            return origin
        limit = max(self.width, self.height)
        for radius in range(1, limit):
            candidates = []
            for dx in range(-radius, radius + 1):
                candidates.append((origin[0] + dx, origin[1] - radius))
                candidates.append((origin[0] + dx, origin[1] + radius))
            for dy in range(-radius + 1, radius):
                candidates.append((origin[0] - radius, origin[1] + dy))
                candidates.append((origin[0] + radius, origin[1] + dy))
            candidates.sort(key=lambda item: self._heuristic(origin, item))
            for candidate in candidates:
                if self._inside(candidate) and candidate not in blocked:
                    return candidate
        return None

    def _astar(
        self, start: Tuple[int, int], goal: Tuple[int, int], blocked: set
    ) -> Optional[List[Tuple[int, int]]]:
        queue = [(self._heuristic(start, goal), 0.0, start)]
        parents = {start: None}
        costs = {start: 0.0}
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost != costs.get(current):
                continue
            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = parents[current]
                return list(reversed(path))

            for dx, dy, step_cost in self._NEIGHBORS:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self._inside(neighbor) or neighbor in blocked:
                    continue
                if dx != 0 and dy != 0:
                    if (current[0] + dx, current[1]) in blocked:
                        continue
                    if (current[0], current[1] + dy) in blocked:
                        continue
                next_cost = cost + step_cost
                if next_cost >= costs.get(neighbor, math.inf):
                    continue
                costs[neighbor] = next_cost
                parents[neighbor] = current
                priority = next_cost + self._heuristic(neighbor, goal)
                heapq.heappush(queue, (priority, next_cost, neighbor))
        return None

    def _smooth(
        self,
        points: Sequence[Tuple[float, float]],
        obstacles: Sequence[Obstacle],
    ) -> List[Tuple[float, float]]:
        if len(points) <= 2:
            return list(points)
        result = [points[0]]
        anchor = 0
        while anchor < len(points) - 1:
            candidate = len(points) - 1
            while candidate > anchor + 1:
                if self.segment_is_free(points[anchor], points[candidate], obstacles):
                    break
                candidate -= 1
            result.append(points[candidate])
            anchor = candidate
        return result

    @staticmethod
    def _densify(
        points: Sequence[Tuple[float, float]],
        max_seg_mm: float = 400.0,
    ) -> List[Tuple[float, float]]:
        result = [points[0]]
        for a, b in zip(points, points[1:]):
            dist = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(math.ceil(dist / max_seg_mm)))
            for i in range(1, n + 1):
                t = i / n
                result.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        return result

    def _to_cell(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        return (
            int(round((x_mm - self.x_min) / self.resolution_mm)),
            int(round((y_mm - self.y_min) / self.resolution_mm)),
        )

    def _to_world(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        return (
            self.x_min + cell[0] * self.resolution_mm,
            self.y_min + cell[1] * self.resolution_mm,
        )

    def _inside(self, cell: Tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    @staticmethod
    def _heuristic(first: Tuple[int, int], second: Tuple[int, int]) -> float:
        dx = abs(first[0] - second[0])
        dy = abs(first[1] - second[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    @staticmethod
    def _point_segment_distance(
        x: float,
        y: float,
        first: Tuple[float, float],
        second: Tuple[float, float],
    ) -> float:
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length_squared = dx * dx + dy * dy
        if length_squared == 0.0:
            return math.hypot(x - first[0], y - first[1])
        ratio = ((x - first[0]) * dx + (y - first[1]) * dy) / length_squared
        ratio = max(0.0, min(1.0, ratio))
        nearest_x = first[0] + ratio * dx
        nearest_y = first[1] + ratio * dy
        return math.hypot(x - nearest_x, y - nearest_y)
