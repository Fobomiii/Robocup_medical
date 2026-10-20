"""Load only the calibrated named goals used by the real Nav2 dispatcher."""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

import yaml


@dataclass(frozen=True)
class FieldGoal:
    goal_id: int
    name: str
    label: str
    x_mm: float
    y_mm: float
    yaw_deg: float


def load_field_goals(config_path: str) -> Tuple[Dict[str, FieldGoal], Dict[int, FieldGoal]]:
    path = Path(config_path).expanduser()
    with path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)

    goals_by_name: Dict[str, FieldGoal] = {}
    goals_by_id: Dict[int, FieldGoal] = {}
    for name, value in raw.get("goals", {}).items():
        goal = FieldGoal(
            goal_id=int(value["id"]),
            name=str(name),
            label=str(value.get("label", name)),
            x_mm=float(value["x_mm"]),
            y_mm=float(value["y_mm"]),
            yaw_deg=float(value.get("yaw_deg", 0.0)),
        )
        if goal.goal_id <= 0:
            raise ValueError(f"goal {name} has invalid id {goal.goal_id}")
        if goal.goal_id in goals_by_id:
            other = goals_by_id[goal.goal_id].name
            raise ValueError(f"goals {other} and {name} share id {goal.goal_id}")
        goals_by_name[goal.name] = goal
        goals_by_id[goal.goal_id] = goal

    if not goals_by_id:
        raise ValueError(f"no goals found in {path}")
    return goals_by_name, goals_by_id
