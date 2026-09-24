"""Configuration and geometry helpers for nurse-station QR acquisition."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Tuple

import yaml


@dataclass(frozen=True)
class NurseScanZone:
    min_x_mm: float
    max_x_mm: float
    min_y_mm: float
    max_y_mm: float

    def contains(self, x_mm: float, y_mm: float) -> bool:
        return (
            self.min_x_mm <= x_mm <= self.max_x_mm
            and self.min_y_mm <= y_mm <= self.max_y_mm
        )


@dataclass(frozen=True)
class NurseScanViewpoint:
    name: str
    x_mm: float
    y_mm: float


@dataclass(frozen=True)
class NurseScanConfig:
    zone: NurseScanZone
    qr_x_mm: float
    qr_y_mm: float
    dwell_s: float
    viewpoint_timeout_s: float
    viewpoints: Tuple[NurseScanViewpoint, ...]


def field_yaw_toward(
    x_mm: float, y_mm: float, target_x_mm: float, target_y_mm: float
) -> float:
    """Return field yaw where 0 deg faces +Y and positive turns toward +X."""
    return math.degrees(math.atan2(target_x_mm - x_mm, target_y_mm - y_mm))


def next_nurse_viewpoint_index(current_index: int, viewpoint_count: int) -> int:
    """Visit center once, then continuously alternate the side viewpoints."""
    if viewpoint_count <= 0:
        raise ValueError("nurse scan requires at least one viewpoint")
    if current_index < 0:
        raise ValueError("nurse viewpoint index cannot be negative")
    if current_index < viewpoint_count:
        return current_index
    return 1 if viewpoint_count > 1 else 0


def load_nurse_scan_config(config_path: str) -> NurseScanConfig:
    path = Path(config_path).expanduser()
    with path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)

    value = raw.get("nurse_scan", {})
    zone_value = value.get("zone", {})
    qr_value = value.get("qr_target", {})
    viewpoints_value = value.get("viewpoints", ())

    zone = NurseScanZone(
        min_x_mm=float(zone_value["min_x_mm"]),
        max_x_mm=float(zone_value["max_x_mm"]),
        min_y_mm=float(zone_value["min_y_mm"]),
        max_y_mm=float(zone_value["max_y_mm"]),
    )
    if zone.min_x_mm >= zone.max_x_mm or zone.min_y_mm >= zone.max_y_mm:
        raise ValueError("nurse scan zone bounds are invalid")

    viewpoints = tuple(
        NurseScanViewpoint(
            name=str(item["name"]),
            x_mm=float(item["x_mm"]),
            y_mm=float(item["y_mm"]),
        )
        for item in viewpoints_value
    )
    if not viewpoints:
        raise ValueError("nurse scan requires at least one fallback viewpoint")

    dwell_s = float(value.get("dwell_s", 1.5))
    viewpoint_timeout_s = float(value.get("viewpoint_timeout_s", 6.0))
    if dwell_s <= 0.0 or viewpoint_timeout_s <= 0.0:
        raise ValueError("nurse scan timing must be positive")

    return NurseScanConfig(
        zone=zone,
        qr_x_mm=float(qr_value["x_mm"]),
        qr_y_mm=float(qr_value["y_mm"]),
        dwell_s=dwell_s,
        viewpoint_timeout_s=viewpoint_timeout_s,
        viewpoints=viewpoints,
    )
