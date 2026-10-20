import json
import math
import sqlite3
import struct
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


ROOT = Path(__file__).parent / "bed13_full_20261008_234017"
DB = ROOT / "rosbag" / "rosbag_0.db3"
START_NS = 1791474019243022966
FAILURE_TIMES_S = (34.521926, 37.841707, 46.342523)


class CdrReader:
    def __init__(self, data):
        self.data = memoryview(data)
        self.offset = 4  # CDR encapsulation

    def align(self, size):
        # Fast-CDR alignment is relative to the payload following the
        # four-byte encapsulation header, not the SQLite blob start.
        self.offset += (-(self.offset - 4)) % size

    def u32(self):
        self.align(4)
        value = struct.unpack_from("<I", self.data, self.offset)[0]
        self.offset += 4
        return value

    def i32(self):
        self.align(4)
        value = struct.unpack_from("<i", self.data, self.offset)[0]
        self.offset += 4
        return value

    def f64(self):
        self.align(8)
        value = struct.unpack_from("<d", self.data, self.offset)[0]
        self.offset += 8
        return value

    def string(self):
        size = self.u32()
        raw = bytes(self.data[self.offset:self.offset + size])
        self.offset += size
        return raw.rstrip(b"\0").decode("utf-8")

    def header(self):
        stamp = self.i32() + self.u32() / 1.0e9
        return stamp, self.string()

    def pose_stamped(self):
        self.header()
        return tuple(self.f64() for _ in range(7))

    def path(self):
        stamp, frame = self.header()
        count = self.u32()
        poses = [self.pose_stamped() for _ in range(count)]
        return stamp, frame, np.asarray([(p[0], p[1]) for p in poses])


def load_paths(connection, topic):
    rows = connection.execute(
        """
        SELECT m.timestamp, m.data
        FROM messages m JOIN topics t ON t.id = m.topic_id
        WHERE t.name = ? ORDER BY m.timestamp
        """,
        (topic,),
    )
    result = []
    for timestamp, data in rows:
        stamp, frame, points = CdrReader(data).path()
        result.append({
            "time": (timestamp - START_NS) / 1.0e9,
            "stamp": stamp,
            "frame": frame,
            "points": points,
        })
    return result


def route_name(time_s):
    if time_s < 22.0:
        return "nurse"
    if time_s < 32.0:
        return "bed1"
    if time_s < 43.0:
        return "bed3"
    return "home"


def path_length(points):
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def turn_metrics(points):
    delta = np.diff(points, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    keep = lengths > 1.0e-6
    delta = delta[keep]
    lengths = lengths[keep]
    if len(delta) < 2:
        return {"max_turn_deg": 0.0, "p95_curvature": 0.0,
                "max_curvature": 0.0, "turns_gt_5": 0, "turns_gt_10": 0}
    headings = np.arctan2(delta[:, 1], delta[:, 0])
    turns = np.abs(np.arctan2(
        np.sin(np.diff(headings)), np.cos(np.diff(headings))))
    spacing = 0.5 * (lengths[:-1] + lengths[1:])
    curvature = turns / np.maximum(spacing, 1.0e-6)
    return {
        "max_turn_deg": float(np.degrees(turns).max()),
        "p95_curvature": float(np.percentile(curvature, 95)),
        "max_curvature": float(curvature.max()),
        "turns_gt_5": int(np.count_nonzero(np.degrees(turns) > 5.0)),
        "turns_gt_10": int(np.count_nonzero(np.degrees(turns) > 10.0)),
    }


def point_to_polyline_distances(points, line):
    if len(points) == 0 or len(line) < 2:
        return np.full(len(points), np.inf)
    a = line[:-1]
    ab = line[1:] - a
    denominator = np.einsum("ij,ij->i", ab, ab)
    result = []
    for point in points:
        ap = point - a
        t = np.divide(
            np.einsum("ij,ij->i", ap, ab), denominator,
            out=np.zeros_like(denominator), where=denominator > 1.0e-12)
        t = np.clip(t, 0.0, 1.0)
        projections = a + t[:, None] * ab
        result.append(np.linalg.norm(projections - point, axis=1).min())
    return np.asarray(result)


def path_distance(points, candidate):
    distances = point_to_polyline_distances(points, candidate)
    return float(np.sqrt(np.mean(np.square(distances))))


def round_metrics(metrics):
    return {key: round(value, 5) if isinstance(value, float) else value
            for key, value in metrics.items()}


def main():
    connection = sqlite3.connect(f"file:{DB.as_posix()}?mode=ro", uri=True)
    raw_all = load_paths(connection, "/plan")
    smooth_all = load_paths(connection, "/plan_smoothed")
    controller_all = load_paths(connection, "/transformed_global_plan")

    pairs = []
    for smooth in smooth_all:
        eligible = [raw for raw in raw_all
                    if 0.0 <= smooth["time"] - raw["time"] <= 0.15]
        if not eligible:
            raise RuntimeError(f"No raw path for smooth path at {smooth['time']}")
        raw = eligible[-1]
        if len(raw["points"]) != len(smooth["points"]):
            raise RuntimeError("Smoother changed pose count unexpectedly")
        displacement = np.linalg.norm(smooth["points"] - raw["points"], axis=1)
        raw_turn = turn_metrics(raw["points"])
        smooth_turn = turn_metrics(smooth["points"])
        failed = min(abs(smooth["time"] - value) for value in FAILURE_TIMES_S) < 0.01

        # FollowPath consumes the BT output on the next controller tick. Skip
        # the first 70 ms so a sample from the preceding plan is not mistaken
        # for the just-published candidate.
        later_controller = [path for path in controller_all
                            if 0.07 <= path["time"] - smooth["time"] <= 0.25]
        controller = later_controller[0] if later_controller else None
        controller_raw_rms = None
        controller_smooth_rms = None
        if controller is not None:
            controller_raw_rms = path_distance(controller["points"], raw["points"])
            controller_smooth_rms = path_distance(controller["points"], smooth["points"])

        pairs.append({
            "route": route_name(smooth["time"]),
            "time": smooth["time"],
            "failed_collision_check": failed,
            "poses": len(smooth["points"]),
            "delay_ms": (smooth["time"] - raw["time"]) * 1000.0,
            "raw_length": path_length(raw["points"]),
            "smooth_length": path_length(smooth["points"]),
            "rms_displacement": float(np.sqrt(np.mean(np.square(displacement)))),
            "max_displacement": float(displacement.max(initial=0.0)),
            "raw_max_turn_deg": raw_turn["max_turn_deg"],
            "smooth_max_turn_deg": smooth_turn["max_turn_deg"],
            "raw_p95_curvature": raw_turn["p95_curvature"],
            "smooth_p95_curvature": smooth_turn["p95_curvature"],
            "raw_turns_gt_5": raw_turn["turns_gt_5"],
            "smooth_turns_gt_5": smooth_turn["turns_gt_5"],
            "controller_raw_rms": controller_raw_rms,
            "controller_smooth_rms": controller_smooth_rms,
        })

    print("PAIR_METRICS")
    for item in pairs:
        print(json.dumps(round_metrics(item), ensure_ascii=False))

    print("\nROUTE_SUMMARY")
    for route in ("nurse", "bed1", "bed3", "home"):
        selected = [item for item in pairs if item["route"] == route]
        successful = [item for item in selected if not item["failed_collision_check"]]
        controller_evidence = [item for item in selected
                               if item["controller_raw_rms"] is not None]
        summary = {
            "route": route,
            "calls": len(selected),
            "successes": len(successful),
            "collision_rejections": len(selected) - len(successful),
            "median_delay_ms": float(np.median([x["delay_ms"] for x in selected])),
            "median_rms_displacement_m": float(np.median(
                [x["rms_displacement"] for x in successful])),
            "max_displacement_m": max(
                [x["max_displacement"] for x in successful], default=0.0),
            "median_raw_max_turn_deg": float(np.median(
                [x["raw_max_turn_deg"] for x in successful])),
            "median_smooth_max_turn_deg": float(np.median(
                [x["smooth_max_turn_deg"] for x in successful])),
            "median_raw_p95_curvature": float(np.median(
                [x["raw_p95_curvature"] for x in successful])),
            "median_smooth_p95_curvature": float(np.median(
                [x["smooth_p95_curvature"] for x in successful])),
            "controller_closer_to_smooth": sum(
                x["controller_smooth_rms"] + 0.001 < x["controller_raw_rms"]
                for x in controller_evidence),
            "controller_closer_to_raw": sum(
                x["controller_raw_rms"] + 0.001 < x["controller_smooth_rms"]
                for x in controller_evidence),
            "controller_ties": sum(
                abs(x["controller_raw_rms"] - x["controller_smooth_rms"]) <= 0.001
                for x in controller_evidence),
        }
        print(json.dumps(round_metrics(summary), ensure_ascii=False))

    success = [item for item in pairs if not item["failed_collision_check"]]
    overall = {
        "raw_paths": len(raw_all),
        "smoothed_paths": len(smooth_all),
        "controller_paths": len(controller_all),
        "runtime_raw_paths": len(pairs),
        "successful_smoothing": len(success),
        "collision_rejections": len(pairs) - len(success),
        "median_delay_ms": float(np.median([x["delay_ms"] for x in pairs])),
        "median_rms_displacement_m": float(np.median(
            [x["rms_displacement"] for x in success])),
        "max_displacement_m": max(x["max_displacement"] for x in success),
        "median_raw_max_turn_deg": float(np.median(
            [x["raw_max_turn_deg"] for x in success])),
        "median_smooth_max_turn_deg": float(np.median(
            [x["smooth_max_turn_deg"] for x in success])),
    }
    print("\nOVERALL")
    print(json.dumps(round_metrics(overall), ensure_ascii=False, indent=2))

    timeline = [json.loads(line) for line in
                (ROOT / "timeline.jsonl").read_text(encoding="utf-8").splitlines()]
    corner_by_state = defaultdict(list)
    wheel_by_state = defaultdict(list)
    for row in timeline:
        if row.get("label") == "CORNER":
            corner_by_state[row.get("task_state")].append(json.loads(row["data"]))
        if row.get("kind") == "odometry" and row.get("label") == "WHEEL_ODOM":
            wheel_by_state[row.get("task_state")].append(
                (row["mono_s"], row.get("speed", 0.0)))

    print("\nRUNTIME_SUMMARY")
    for state, name in ((3, "bed1"), (6, "bed3"), (9, "home")):
        corners = corner_by_state[state]
        wheels = wheel_by_state[state]
        moving = [speed for _, speed in wheels if speed > 0.05]
        print(json.dumps({
            "route": name,
            "state_duration_s": round(wheels[-1][0] - wheels[0][0], 3),
            "peak_measured_speed_m_s": round(max(x[1] for x in wheels), 3),
            "mean_moving_speed_m_s": round(float(np.mean(moving)), 3),
            "corner_status_samples": len(corners),
            "corner_limited_samples": sum(bool(x.get("limited")) for x in corners),
            "curvature_limited_samples": sum(
                x.get("corner_limit_m_s", 3.0) < 2.999 for x in corners),
            "blind_zone_limited_samples": sum(
                bool(x.get("blind_zone_limited")) for x in corners),
            "corner_sources": dict(Counter(x.get("source") for x in corners)),
            "minimum_reported_limit_m_s": round(
                min((x.get("limit_m_s", 3.0) for x in corners), default=3.0), 3),
            "minimum_curvature_limit_m_s": round(
                min((x.get("corner_limit_m_s", 3.0) for x in corners), default=3.0), 3),
        }, ensure_ascii=False))

    journal = (ROOT / "journal.log").read_text(
        encoding="utf-8", errors="replace").splitlines()
    monitor_events = [line for line in journal if "collision_monitor" in line and (
        "Robot to approach" in line or "Robot to stop" in line or
        "normal operation" in line)]
    print(json.dumps({
        "collision_monitor_approach_events": sum(
            "Robot to approach" in line for line in monitor_events),
        "collision_monitor_stop_events": sum(
            "Robot to stop" in line for line in monitor_events),
        "collision_monitor_normal_events": sum(
            "normal operation" in line for line in monitor_events),
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
