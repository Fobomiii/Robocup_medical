"""Guarded OPS XY calibration using known bedside field geometry."""

from dataclasses import dataclass
import math
import statistics
from typing import Dict, Iterable, Optional, Tuple

import yaml

from .nav_protocol import NAV_FOLLOWING, NAV_REACHED, Stp23lTelemetry


SENSOR_BITS = {"a": 0x01, "b": 0x02, "c": 0x04}


@dataclass(frozen=True)
class BedGeometry:
    name: str
    task_states: Tuple[int, ...]
    side_sensor: str
    side_reference_x_mm: float
    front_reference_y_mm: float
    expected_center_side_distance_mm: float
    expected_center_front_distance_mm: float

    @property
    def true_x_mm(self) -> float:
        if self.side_sensor == "c":
            return self.side_reference_x_mm + self.expected_center_side_distance_mm
        return self.side_reference_x_mm - self.expected_center_side_distance_mm

    @property
    def true_y_mm(self) -> float:
        return self.front_reference_y_mm - self.expected_center_front_distance_mm


@dataclass(frozen=True)
class CalibrationConfig:
    enabled: bool
    sensor_radius_mm: float
    samples_required: int
    max_offset_spread_mm: float
    distance_tolerance_mm: float
    yaw_tolerance_deg: float
    max_offset_mm: float
    beds: Tuple[BedGeometry, ...]


@dataclass(frozen=True)
class CalibrationEvent:
    bed: Optional[str]
    state: str
    sample_count: int
    applied: bool = False
    offset_x_mm: float = 0.0
    offset_y_mm: float = 0.0
    measured_x_mm: Optional[float] = None
    measured_y_mm: Optional[float] = None


def load_calibration_config(path: str) -> CalibrationConfig:
    with open(path, encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    section = raw.get("stp23l_calibration", {})
    beds = []
    for name, item in section.get("beds", {}).items():
        side_sensor = str(item["side_sensor"]).lower()
        if side_sensor not in ("a", "c"):
            raise ValueError(f"{name}: side_sensor must be A or C")
        geometry = BedGeometry(
            name=name,
            task_states=tuple(int(value) for value in item["task_states"]),
            side_sensor=side_sensor,
            side_reference_x_mm=float(item["side_reference_x_mm"]),
            front_reference_y_mm=float(item["front_reference_y_mm"]),
            expected_center_side_distance_mm=float(
                item["expected_center_side_distance_mm"]
            ),
            expected_center_front_distance_mm=float(
                item["expected_center_front_distance_mm"]
            ),
        )
        goal = raw.get("goals", {}).get(name)
        if goal is None:
            raise ValueError(f"{name}: matching goal is missing")
        if (
            abs(float(goal["x_mm"]) - geometry.true_x_mm) > 1.0
            or abs(float(goal["y_mm"]) - geometry.true_y_mm) > 1.0
        ):
            raise ValueError(f"{name}: goal does not match STP23L field geometry")
        beds.append(geometry)

    return CalibrationConfig(
        enabled=bool(section.get("enabled", False)),
        sensor_radius_mm=float(section.get("sensor_radius_mm", 153.0)),
        samples_required=max(3, int(section.get("samples_required", 7))),
        max_offset_spread_mm=float(section.get("max_offset_spread_mm", 40.0)),
        distance_tolerance_mm=float(section.get("distance_tolerance_mm", 200.0)),
        yaw_tolerance_deg=float(section.get("yaw_tolerance_deg", 8.0)),
        max_offset_mm=float(section.get("max_offset_mm", 400.0)),
        beds=tuple(beds),
    )


class OpsRangeCalibrator:
    def __init__(self, config: CalibrationConfig) -> None:
        self.config = config
        self.offset_x_mm = 0.0
        self.offset_y_mm = 0.0
        self._samples = []
        self._active_bed: Optional[str] = None
        self._calibrated_beds = set()
        self._by_state: Dict[int, BedGeometry] = {
            state: bed for bed in config.beds for state in bed.task_states
        }

    def corrected_xy(self, raw_x_mm: float, raw_y_mm: float) -> Tuple[float, float]:
        return raw_x_mm + self.offset_x_mm, raw_y_mm + self.offset_y_mm

    @staticmethod
    def _distance(telemetry: Stp23lTelemetry, sensor: str) -> int:
        return int(getattr(telemetry, f"{sensor}_mm"))

    @staticmethod
    def _spread(values: Iterable[float]) -> float:
        values = tuple(values)
        return max(values) - min(values)

    def update(
        self,
        task_state: int,
        nav_status: int,
        raw_x_mm: float,
        raw_y_mm: float,
        yaw_cdeg: int,
        telemetry: Stp23lTelemetry,
    ) -> CalibrationEvent:
        bed = self._by_state.get(int(task_state))
        if not self.config.enabled:
            return CalibrationEvent(None, "disabled", 0)
        if bed is None:
            self._calibrated_beds.clear()
            self._samples.clear()
            self._active_bed = None
            return CalibrationEvent(None, "not_at_calibration_point", 0)
        if self._active_bed != bed.name:
            if self._active_bed is not None:
                self._calibrated_beds.discard(self._active_bed)
            self._samples.clear()
            self._active_bed = bed.name
        if bed.name in self._calibrated_beds:
            return CalibrationEvent(bed.name, "already_calibrated", 0)
        if int(nav_status) not in (NAV_FOLLOWING, NAV_REACHED):
            self._samples.clear()
            return CalibrationEvent(bed.name, "waiting_for_navigation", 0)

        required_mask = SENSOR_BITS["b"] | SENSOR_BITS[bed.side_sensor]
        if telemetry.valid_mask & required_mask != required_mask:
            self._samples.clear()
            return CalibrationEvent(bed.name, "required_sensor_invalid", 0)

        front_mm = self._distance(telemetry, "b")
        side_mm = self._distance(telemetry, bed.side_sensor)
        expected_front_raw = (
            bed.expected_center_front_distance_mm - self.config.sensor_radius_mm
        )
        expected_side_raw = (
            bed.expected_center_side_distance_mm - self.config.sensor_radius_mm
        )
        if (
            abs(front_mm - expected_front_raw) > self.config.distance_tolerance_mm
            or abs(side_mm - expected_side_raw) > self.config.distance_tolerance_mm
        ):
            self._samples.clear()
            return CalibrationEvent(bed.name, "range_outside_geometry_gate", 0)

        yaw_deg = math.remainder(float(yaw_cdeg) / 100.0, 360.0)
        if abs(yaw_deg) > self.config.yaw_tolerance_deg:
            self._samples.clear()
            return CalibrationEvent(bed.name, "yaw_outside_gate", 0)

        projection = math.cos(math.radians(yaw_deg))
        center_front_mm = (front_mm + self.config.sensor_radius_mm) * projection
        center_side_mm = (side_mm + self.config.sensor_radius_mm) * projection
        measured_y_mm = bed.front_reference_y_mm - center_front_mm
        if bed.side_sensor == "c":
            measured_x_mm = bed.side_reference_x_mm + center_side_mm
        else:
            measured_x_mm = bed.side_reference_x_mm - center_side_mm

        candidate_x = measured_x_mm - float(raw_x_mm)
        candidate_y = measured_y_mm - float(raw_y_mm)
        if (
            abs(candidate_x) > self.config.max_offset_mm
            or abs(candidate_y) > self.config.max_offset_mm
        ):
            self._samples.clear()
            return CalibrationEvent(bed.name, "offset_outside_gate", 0)

        self._samples.append(
            (front_mm, side_mm, raw_x_mm, raw_y_mm, candidate_x, candidate_y,
             measured_x_mm, measured_y_mm)
        )
        if len(self._samples) > self.config.samples_required:
            self._samples.pop(0)
        count = len(self._samples)
        if count < self.config.samples_required:
            return CalibrationEvent(bed.name, "collecting", count)

        # The robot may still be approaching the goal. Raw ranges and OPS pose
        # then change together, while the independently estimated OPS offset
        # must remain constant. This permits correction before Nav2 declares
        # success without accepting a moving person as a field reference.
        if (
            self._spread(sample[4] for sample in self._samples)
            > self.config.max_offset_spread_mm
            or self._spread(sample[5] for sample in self._samples)
            > self.config.max_offset_spread_mm
        ):
            self._samples.pop(0)
            return CalibrationEvent(bed.name, "unstable", len(self._samples))

        self.offset_x_mm = statistics.median(sample[4] for sample in self._samples)
        self.offset_y_mm = statistics.median(sample[5] for sample in self._samples)
        final_x = statistics.median(sample[6] for sample in self._samples)
        final_y = statistics.median(sample[7] for sample in self._samples)
        self._calibrated_beds.add(bed.name)
        self._samples.clear()
        return CalibrationEvent(
            bed.name,
            "applied",
            count,
            applied=True,
            offset_x_mm=self.offset_x_mm,
            offset_y_mm=self.offset_y_mm,
            measured_x_mm=final_x,
            measured_y_mm=final_y,
        )
