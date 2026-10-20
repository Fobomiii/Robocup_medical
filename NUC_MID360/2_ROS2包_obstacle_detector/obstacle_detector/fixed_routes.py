"""Load the calibrated field model and select phase-one fixed routes."""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import yaml


@dataclass(frozen=True)
class Waypoint:
    x_mm: float
    y_mm: float
    yaw_deg: float
    speed_mm_s: float


@dataclass(frozen=True)
class Goal:
    goal_id: int
    name: str
    label: str
    x_mm: float
    y_mm: float
    yaw_deg: float


class FixedRouteMap:
    def __init__(self, config_path: str) -> None:
        self.config_path = str(Path(config_path).expanduser())
        with open(self.config_path, "r", encoding="utf-8") as stream:
            self.raw = yaml.safe_load(stream)

        self.goals_by_name: Dict[str, Goal] = {}
        self.goals_by_id: Dict[int, Goal] = {}
        for name, value in self.raw["goals"].items():
            goal = Goal(
                int(value["id"]),
                name,
                str(value.get("label", name)),
                float(value["x_mm"]),
                float(value["y_mm"]),
                float(value.get("yaw_deg", 0.0)),
            )
            self.goals_by_name[name] = goal
            self.goals_by_id[goal.goal_id] = goal

        self.routes: Dict[Tuple[str, str], List[Waypoint]] = {}
        for route in self.raw.get("routes", []):
            points = [
                Waypoint(
                    float(point["x_mm"]),
                    float(point["y_mm"]),
                    float(point.get("yaw_deg", 0.0)),
                    float(point.get("speed_mm_s", 150.0)),
                )
                for point in route["waypoints"]
            ]
            self.routes[(route["from"], route["to"])] = points

    def nearest_goal_name(self, x_mm: float, y_mm: float) -> str:
        return min(
            self.goals_by_name.values(),
            key=lambda goal: (goal.x_mm - x_mm) ** 2 + (goal.y_mm - y_mm) ** 2,
        ).name

    def path_for(self, goal_id: int, x_mm: float, y_mm: float) -> Tuple[str, Goal, List[Waypoint]]:
        try:
            goal = self.goals_by_id[goal_id]
        except KeyError as exc:
            raise ValueError(f"unknown goal id: {goal_id}") from exc

        source = self.nearest_goal_name(x_mm, y_mm)
        points = self.routes.get((source, goal.name))
        if points is None:
            points = [Waypoint(goal.x_mm, goal.y_mm, goal.yaw_deg, 120.0)]
        return source, goal, list(points)
