"""Unit tests for obstacle mapping and collision-free planning."""

import math
import os
import unittest

import numpy as np

from obstacle_detector.fixed_routes import FixedRouteMap
from obstacle_detector.navigation_core import GridPlanner, Obstacle, ObstacleMapper


CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "config", "field_map.yaml"
)


class GridPlannerTest(unittest.TestCase):
    def setUp(self):
        self.route_map = FixedRouteMap(CONFIG)
        self.planner = GridPlanner(
            self.route_map.raw["field"], self.route_map.raw["fixtures"], inflation_mm=0.0
        )

    def assert_path_is_free(self, start, path, obstacles):
        points = [start] + [(point.x_mm, point.y_mm) for point in path]
        for first, second in zip(points, points[1:]):
            self.assertTrue(
                self.planner.segment_is_free(first, second, obstacles),
                f"unsafe segment {first}->{second}",
            )

    def test_bed_to_bed_uses_rear_channel(self):
        goal = self.route_map.goals_by_name["bed3"]
        path = self.planner.plan(-2100.0, 3800.0, goal, [])
        self.assertIsNotNone(path)
        self.assertLessEqual(len(path), 16)
        self.assertLessEqual(min(point.y_mm for point in path[:-1]), 4140.0)
        self.assert_path_is_free((-2100.0, 3800.0), path, [])

    def test_planner_detours_around_random_obstacle(self):
        obstacle = Obstacle(0.0, 3800.0, 180.0)
        goal = self.route_map.goals_by_name["bed3"]
        path = self.planner.plan(-2100.0, 3800.0, goal, [obstacle])
        self.assertIsNotNone(path)
        self.assert_path_is_free((-2100.0, 3800.0), path, [obstacle])
        self.assertFalse(
            self.planner.path_blocked((-2100.0, 3800.0), path, [obstacle])
        )


class ObstacleMapperTest(unittest.TestCase):
    def setUp(self):
        route_map = FixedRouteMap(CONFIG)
        self.mapper = ObstacleMapper(
            route_map.raw["field"],
            route_map.raw["fixtures"],
            confirm_hits=3,
            cluster_min_points=4,
        )

    @staticmethod
    def cluster(forward_m, left_m):
        offsets = np.linspace(-0.035, 0.035, 9)
        return np.column_stack(
            (
                forward_m + offsets,
                left_m + offsets[::-1],
                np.full_like(offsets, 0.20),
            )
        )

    def test_multiple_obstacles_require_three_confirmations(self):
        points = np.vstack((self.cluster(2.5, 1.5), self.cluster(3.0, -1.4)))
        for frame in range(2):
            self.mapper.update(points, 0.0, 0.0, 0.0, float(frame) * 0.1)
            self.assertEqual(self.mapper.obstacles(), [])

        changed = self.mapper.update(points, 0.0, 0.0, 0.0, 0.2)
        obstacles = self.mapper.obstacles()
        self.assertTrue(changed)
        self.assertEqual(len(obstacles), 2)
        expected = [(-1500.0, 2500.0), (1400.0, 3000.0)]
        actual = sorted((item.x_mm, item.y_mm) for item in obstacles)
        for measured, target in zip(actual, expected):
            self.assertLess(math.dist(measured, target), 80.0)

    def test_fixed_fixture_points_are_rejected(self):
        # Robot at the origin, yaw=0: this return lands in the nurse station.
        points = self.cluster(2.3, 0.0)
        for frame in range(4):
            self.mapper.update(points, 0.0, 0.0, 0.0, frame * 0.1)
        self.assertEqual(self.mapper.obstacles(), [])

    def test_map_is_limited_to_five_obstacles(self):
        world_points = (
            (-2200.0, 900.0),
            (-1200.0, 2900.0),
            (1100.0, 900.0),
            (1900.0, 2500.0),
            (0.0, 3500.0),
            (2700.0, 1200.0),
        )
        points = np.vstack(
            [self.cluster(y / 1000.0, -x / 1000.0) for x, y in world_points]
        )
        for frame in range(3):
            self.mapper.update(points, 0.0, 0.0, 0.0, frame * 0.1)
        self.assertEqual(len(self.mapper.obstacles()), 5)

    def test_clockwise_yaw_transform(self):
        points = self.cluster(1.0, 0.0)
        for frame in range(3):
            self.mapper.update(points, -1000.0, 1000.0, 90.0, frame * 0.1)
        obstacle = self.mapper.obstacles()[0]
        self.assertAlmostEqual(obstacle.x_mm, 0.0, delta=80.0)
        self.assertAlmostEqual(obstacle.y_mm, 1000.0, delta=80.0)


if __name__ == "__main__":
    unittest.main()
