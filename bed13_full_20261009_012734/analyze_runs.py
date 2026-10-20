import json
import math
import re
import sqlite3
import statistics
import struct
import sys
from collections import Counter, defaultdict
from pathlib import Path


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parent
DB = ROOT / "rosbag" / "rosbag_0.db3"

STATE_NAMES = {
    0: "init",
    1: "nav_nurse",
    2: "scan_order",
    3: "nav_bed1",
    4: "scan_bed1",
    5: "dispense_bed1",
    6: "nav_bed3",
    7: "scan_bed3",
    8: "dispense_bed3",
    9: "nav_home",
    10: "complete",
    11: "nav_error",
    12: "dock_bed1",
    13: "dock_bed3",
    14: "wait_start",
    15: "wait_bed1_start",
    16: "wait_bed3_start",
}


class CdrReader:
    def __init__(self, data):
        self.data = memoryview(data)
        self.offset = 4

    def align(self, size):
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
        return stamp, frame, [(pose[0], pose[1]) for pose in poses]


def percentile(values, q):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * q
    low = math.floor(index)
    high = math.ceil(index)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - index) + ordered[high] * (index - low)


def parse_journal():
    pattern = re.compile(r"\[(\d{10}\.\d+)\].*?: (.*)$")
    events = []
    for line in (ROOT / "journal.log").read_text(
            encoding="utf-8", errors="replace").splitlines():
        match = pattern.search(line)
        if match:
            events.append((float(match.group(1)), match.group(2), line))
    return events


def parse_timeline():
    with (ROOT / "timeline.jsonl").open(encoding="utf-8") as source:
        return [json.loads(line) for line in source]


def load_paths(topic):
    connection = sqlite3.connect(f"file:{DB.as_posix()}?mode=ro", uri=True)
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
            "time": timestamp / 1.0e9,
            "stamp": stamp,
            "frame": frame,
            "points": points,
        })
    connection.close()
    return result


def path_length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


def path_turn_metrics(points):
    segments = []
    for first, second in zip(points, points[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length = math.hypot(dx, dy)
        if length > 1.0e-6:
            segments.append((math.atan2(dy, dx), length))
    turns = []
    curvatures = []
    for first, second in zip(segments, segments[1:]):
        turn = abs(math.atan2(
            math.sin(second[0] - first[0]), math.cos(second[0] - first[0])))
        turns.append(math.degrees(turn))
        curvatures.append(turn / max(0.5 * (first[1] + second[1]), 1.0e-6))
    longest_arc_like_run = 0
    current_arc_like_run = 0
    for turn in turns:
        if 0.5 <= turn <= 8.0:
            current_arc_like_run += 1
            longest_arc_like_run = max(longest_arc_like_run, current_arc_like_run)
        else:
            current_arc_like_run = 0
    return {
        "max_turn_deg": max(turns, default=0.0),
        "p95_curvature": percentile(curvatures, 0.95),
        "turns_gt_5": sum(value > 5.0 for value in turns),
        "turns_gt_10": sum(value > 10.0 for value in turns),
        "longest_arc_like_run": longest_arc_like_run,
    }


def point_polyline_distance(point, line):
    best = float("inf")
    for first, second in zip(line, line[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        denominator = dx * dx + dy * dy
        if denominator <= 1.0e-12:
            projection = first
        else:
            ratio = ((point[0] - first[0]) * dx +
                     (point[1] - first[1]) * dy) / denominator
            ratio = min(1.0, max(0.0, ratio))
            projection = (first[0] + ratio * dx, first[1] + ratio * dy)
        best = min(best, math.hypot(
            point[0] - projection[0], point[1] - projection[1]))
    return best


def path_rms_distance(points, line):
    if not points or len(line) < 2:
        return float("inf")
    distances = [point_polyline_distance(point, line) for point in points]
    return math.sqrt(statistics.mean(value * value for value in distances))


def first_event(events, start, end, text):
    return next((event for event in events
                 if start <= event[0] < end and text in event[1]), None)


def events_between(events, start, end, text):
    return [event for event in events
            if start <= event[0] < end and text in event[1]]


def integrate_motion(samples, start, end, threshold=0.05):
    selected = sorted(
        (float(row["wall_ns"]) / 1.0e9, float(row.get("speed", 0.0)))
        for row in samples
        if start <= float(row["wall_ns"]) / 1.0e9 <= end
    )
    if not selected:
        return {}
    intervals = []
    distance = 0.0
    moving_time = 0.0
    time_above = Counter()
    for first, second in zip(selected, selected[1:]):
        dt = min(second[0] - first[0], 0.1)
        if dt <= 0.0:
            continue
        speed = 0.5 * (first[1] + second[1])
        distance += speed * dt
        if speed > threshold:
            moving_time += dt
        for limit in (0.5, 1.0, 1.5, 2.0):
            if speed > limit:
                time_above[limit] += dt
        intervals.append((first[0], second[0], speed))
    moving_samples = [(time, speed) for time, speed in selected if speed > threshold]
    first_move = moving_samples[0][0] if moving_samples else None
    last_move = moving_samples[-1][0] if moving_samples else None
    longest_stop = 0.0
    if first_move is not None and last_move is not None:
        stop_start = None
        for time, speed in selected:
            if time < first_move or time > last_move:
                continue
            if speed <= threshold and stop_start is None:
                stop_start = time
            elif speed > threshold and stop_start is not None:
                longest_stop = max(longest_stop, time - stop_start)
                stop_start = None
    return {
        "samples": len(selected),
        "duration_s": end - start,
        "first_move_delay_s": None if first_move is None else first_move - start,
        "last_move_before_end_s": None if last_move is None else end - last_move,
        "moving_time_s": moving_time,
        "distance_integral_m": distance,
        "mean_while_moving_m_s": distance / moving_time if moving_time else 0.0,
        "peak_m_s": max(speed for _, speed in selected),
        "p95_m_s": percentile([speed for _, speed in selected], 0.95),
        "longest_internal_stop_s": longest_stop,
        "time_above_0_5_s": time_above[0.5],
        "time_above_1_0_s": time_above[1.0],
        "time_above_1_5_s": time_above[1.5],
        "time_above_2_0_s": time_above[2.0],
    }


def sample_metrics(samples, start, end, threshold=0.05):
    selected = sorted(
        (float(row["wall_ns"]) / 1.0e9, float(row.get("speed", 0.0)))
        for row in samples
        if start <= float(row["wall_ns"]) / 1.0e9 <= end
    )
    active = [(time, speed) for time, speed in selected if speed > threshold]
    return {
        "samples": len(selected),
        "first_active_delay_s": active[0][0] - start if active else None,
        "last_active_before_end_s": end - active[-1][0] if active else None,
        "peak_m_s": max((speed for _, speed in selected), default=0.0),
        "mean_active_m_s": statistics.mean(
            [speed for _, speed in active]) if active else 0.0,
    }


def wheel_start_metrics(samples, start, end, rpm_threshold=20):
    selected = sorted(
        (time, payload) for time, payload in samples if start <= time <= end)
    target = [time for time, payload in selected
              if max((abs(int(value)) for value in payload.get("target_rpm", [])),
                     default=0) > rpm_threshold]
    measured = [time for time, payload in selected
                if max((abs(int(value)) for value in payload.get("measured_rpm", [])),
                       default=0) > rpm_threshold]
    return {
        "first_target_rpm_delay_s": target[0] - start if target else None,
        "first_measured_rpm_delay_s": measured[0] - start if measured else None,
        "target_to_measured_delay_s": (
            measured[0] - target[0] if target and measured else None),
    }


def monitor_intervals(events):
    result = []
    active = None
    for time, message, _ in events:
        if "Robot to approach" in message:
            if active is None:
                active = time
        elif "Robot to continue normal operation" in message and active is not None:
            result.append((active, time))
            active = None
    return result


def overlap_duration(intervals, start, end):
    return sum(max(0.0, min(finish, end) - max(begin, start))
               for begin, finish in intervals)


def round_values(value):
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        return round(value, 4)
    if isinstance(value, dict):
        return {key: round_values(item) for key, item in value.items()}
    if isinstance(value, list):
        return [round_values(item) for item in value]
    return value


def main():
    journal = parse_journal()
    timeline = parse_timeline()
    bag_end = max(float(row["wall_ns"]) / 1.0e9 for row in timeline)
    missions = [(time, int(re.search(r"epoch=(\d+)", message).group(1)))
                for time, message, _ in journal
                if "New medical mission epoch=" in message]
    if not missions:
        first_dispatch = next(
            time for time, message, _ in journal
            if message.startswith("Dispatching nurse"))
        missions = [(first_dispatch, 0)]
    runs = []
    for index, (start, epoch) in enumerate(missions):
        end = missions[index + 1][0] if index + 1 < len(missions) else bag_end + 1.0
        runs.append({"run": index + 1, "epoch": epoch, "start": start, "end": end})

    state_transitions = []
    previous = object()
    for row in timeline:
        state = row.get("task_state")
        if state != previous:
            state_transitions.append((float(row["wall_ns"]) / 1.0e9, state))
            previous = state

    wheel = [row for row in timeline
             if row.get("kind") == "odometry" and row.get("label") == "WHEEL_ODOM"]
    twists = defaultdict(list)
    for row in timeline:
        if row.get("kind") == "twist":
            twists[row.get("layer")].append(row)
    corner = []
    for row in timeline:
        if row.get("label") != "CORNER":
            continue
        try:
            payload = json.loads(row["data"])
        except (KeyError, json.JSONDecodeError):
            continue
        corner.append((float(row["wall_ns"]) / 1.0e9, payload))

    wheel_diagnostics = []
    safety_status = []
    for row in timeline:
        if row.get("label") != "WHEEL_DIAG":
            continue
        try:
            payload = json.loads(row["data"])
        except (KeyError, json.JSONDecodeError):
            continue
        wheel_diagnostics.append((float(row["wall_ns"]) / 1.0e9, payload))
    for row in timeline:
        if row.get("label") != "LIDAR_SAFETY_FILTER":
            continue
        try:
            payload = json.loads(row["data"])
        except (KeyError, json.JSONDecodeError):
            continue
        safety_status.append((float(row["wall_ns"]) / 1.0e9, payload))

    approaches = monitor_intervals(journal)
    raw_paths = load_paths("/plan")
    smooth_paths = load_paths("/plan_smoothed")
    controller_paths = load_paths("/transformed_global_plan")

    print("STATE_TRANSITIONS")
    for run in runs:
        selected = [(time, state) for time, state in state_transitions
                    if run["start"] - 0.1 <= time < run["end"]]
        print(json.dumps({
            "run": run["run"],
            "transitions": [
                {"t_from_mission_s": time - run["start"], "state": state,
                 "name": STATE_NAMES.get(state, "unknown")}
                for time, state in selected
            ],
        }, ensure_ascii=False))

    all_segment_summaries = []
    print("\nRUNS")
    for run in runs:
        dispatches = []
        for time, message, _ in journal:
            if not (run["start"] <= time < run["end"] and
                    message.startswith("Dispatching ")):
                continue
            name = re.search(r"Dispatching (\S+)", message).group(1)
            route = "nurse" if name.startswith("nurse") else name
            dispatches.append((time, route, message))

        run_summary = {
            "run": run["run"],
            "epoch": run["epoch"],
            "mission_start": run["start"],
            "segments": [],
        }
        for segment_index, (start, route, _) in enumerate(dispatches):
            next_start = (dispatches[segment_index + 1][0]
                          if segment_index + 1 < len(dispatches) else run["end"])
            if route == "nurse":
                finish_event = first_event(
                    journal, start, next_start, "Confirmed scan context=1")
                ack_event = first_event(
                    journal, start, next_start, "accepted scan")
            else:
                finish_event = first_event(
                    journal, start, next_start, "Navigation succeeded")
                ack_event = None
            finish = finish_event[0] if finish_event else next_start
            motion = integrate_motion(wheel, start, finish)
            corners = [payload for time, payload in corner if start <= time <= finish]
            diagnostics = [payload for time, payload in wheel_diagnostics
                           if start <= time <= finish]
            summary = {
                "route": route,
                "start": start,
                "finish": finish,
                "duration_s": finish - start,
                "ack_delay_after_detection_s": (
                    ack_event[0] - finish if ack_event is not None else None),
                "motion": motion,
                "command_chain": {
                    layer: sample_metrics(twists[layer], start, finish)
                    for layer in ("RAW", "SMOOTH", "HOME", "SAFE")
                },
                "wheel_start": wheel_start_metrics(
                    wheel_diagnostics, start, finish),
                "approach_entries": len(events_between(
                    journal, start, finish, "Robot to approach")),
                "approach_active_s": overlap_duration(approaches, start, finish),
                "collision_stop_entries": len(events_between(
                    journal, start, finish, "Robot to stop")),
                "smoother_collision_rejections": len(events_between(
                    journal, start, finish, "Smoothed path leads to a collision")),
                "planner_failures": len(events_between(
                    journal, start, finish, "failed to generate a valid path")),
                "corner_samples": len(corners),
                "corner_limited_samples": sum(bool(item.get("limited")) for item in corners),
                "curvature_limited_samples": sum(
                    float(item.get("corner_limit_m_s", 3.0)) < 2.999 for item in corners),
                "blind_limited_samples": sum(
                    bool(item.get("blind_zone_limited")) for item in corners),
                "minimum_limit_m_s": min(
                    (float(item.get("limit_m_s", 3.0)) for item in corners), default=3.0),
                "minimum_curvature_limit_m_s": min(
                    (float(item.get("corner_limit_m_s", 3.0)) for item in corners), default=3.0),
                "current_saturation_samples": sum(
                    bool(item.get("current_saturation_mask")) for item in diagnostics),
                "offline_wheel_samples": sum(
                    int(item.get("online_mask", 0)) != 15 for item in diagnostics),
            }
            run_summary["segments"].append(summary)
            all_segment_summaries.append({"run": run["run"], **summary})

        home = next((item for item in run_summary["segments"]
                     if item["route"] == "home"), None)
        nurse = next((item for item in run_summary["segments"]
                      if item["route"] == "nurse"), None)
        complete_transition = next(
            (time for time, state in state_transitions
             if state == 10 and run["start"] <= time < run["end"]), None)
        competition_start = next(
            (time for time, state in state_transitions
             if state == 1 and run["start"] <= time < run["end"]), None)
        run_summary["mission_to_home_nav_success_s"] = (
            home["finish"] - run["start"] if home else None)
        run_summary["nurse_dispatch_to_home_nav_success_s"] = (
            home["finish"] - nurse["start"] if home and nurse else None)
        run_summary["mission_to_complete_state_s"] = (
            complete_transition - run["start"] if complete_transition else None)
        run_summary["competition_state_1_to_complete_s"] = (
            complete_transition - competition_start
            if complete_transition and competition_start else None)
        run_summary["startup_to_nurse_dispatch_s"] = (
            nurse["start"] - run["start"] if nurse else None)
        competition_end = complete_transition or (home["finish"] if home else run["end"])
        run_summary["telemetry_losses_during_competition"] = len(events_between(
            journal, competition_start or run["start"], competition_end,
            "telemetry lost"))
        run_summary["navigation_recoveries"] = sum(
            len(events_between(journal, run["start"], run["end"], text))
            for text in ("Spin", "BackUp", "Wait"))
        safety_samples = [payload for time, payload in safety_status
                          if (competition_start or run["start"]) <= time <= competition_end]
        run_summary["safety_filter"] = {
            "samples": len(safety_samples),
            "diagnostic_stop_band_positive_samples": sum(
                int(item.get("points_in_stop_zone", 0)) > 0 for item in safety_samples),
            "maximum_points_in_diagnostic_stop_band": max(
                (int(item.get("points_in_stop_zone", 0)) for item in safety_samples),
                default=0),
            "minimum_nearest_output_xy_m": min(
                (float(item.get("nearest_output_xy_m", 99.0)) for item in safety_samples),
                default=99.0),
            "maximum_processing_ms": max(
                (float(item.get("processing_ms", 0.0)) for item in safety_samples),
                default=0.0),
            "maximum_output_age_s": max(
                (float(item.get("output_age_s", 0.0)) for item in safety_samples),
                default=0.0),
        }
        print(json.dumps(round_values(run_summary), ensure_ascii=False))

    print("\nROUTE_AGGREGATES")
    for route in ("nurse", "bed1", "bed3", "home"):
        selected = [item for item in all_segment_summaries if item["route"] == route]
        fields = {}
        for key in ("duration_s", "approach_active_s"):
            values = [float(item[key]) for item in selected]
            fields[key] = {
                "mean": statistics.mean(values),
                "min": min(values),
                "max": max(values),
                "range": max(values) - min(values),
            }
        for key in ("peak_m_s", "mean_while_moving_m_s", "moving_time_s",
                    "distance_integral_m", "longest_internal_stop_s"):
            values = [float(item["motion"].get(key, 0.0)) for item in selected]
            fields[key] = {
                "mean": statistics.mean(values),
                "min": min(values),
                "max": max(values),
            }
        fields["approach_entries_total"] = sum(item["approach_entries"] for item in selected)
        fields["collision_stop_entries_total"] = sum(
            item["collision_stop_entries"] for item in selected)
        fields["smoother_collision_rejections_total"] = sum(
            item["smoother_collision_rejections"] for item in selected)
        fields["planner_failures_total"] = sum(item["planner_failures"] for item in selected)
        fields["curvature_limited_samples_total"] = sum(
            item["curvature_limited_samples"] for item in selected)
        fields["blind_limited_samples_total"] = sum(
            item["blind_limited_samples"] for item in selected)
        fields["minimum_curvature_limit_m_s"] = min(
            item["minimum_curvature_limit_m_s"] for item in selected)
        print(json.dumps(round_values({"route": route, **fields}), ensure_ascii=False))

    path_segments = []
    for item in all_segment_summaries:
        raw = [path for path in raw_paths if item["start"] <= path["time"] <= item["finish"]]
        smooth = [path for path in smooth_paths
                  if item["start"] <= path["time"] <= item["finish"]]
        controller = [path for path in controller_paths
                      if item["start"] <= path["time"] <= item["finish"]]
        pairs = []
        for candidate in smooth:
            preceding = [path for path in raw
                         if 0.0 <= candidate["time"] - path["time"] <= 0.2]
            if not preceding:
                continue
            source = preceding[-1]
            later = [path for path in controller
                     if 0.07 <= path["time"] - candidate["time"] <= 0.25]
            consumed = later[0] if later else None
            raw_metrics = path_turn_metrics(source["points"])
            smooth_metrics = path_turn_metrics(candidate["points"])
            pairs.append({
                "raw": source,
                "smooth": candidate,
                "consumed": consumed,
                "raw_metrics": raw_metrics,
                "smooth_metrics": smooth_metrics,
            })
        successful_raw_ids = {id(pair["raw"]) for pair in pairs}
        path_segments.append({
            "run": item["run"],
            "route": item["route"],
            "raw_paths": len(raw),
            "smoothed_paths": len(smooth),
            "paired_paths": len(pairs),
            "unpaired_raw_paths": len([path for path in raw if id(path) not in successful_raw_ids]),
            "median_raw_length_m": statistics.median(
                [path_length(pair["raw"]["points"]) for pair in pairs]) if pairs else 0.0,
            "median_smooth_length_m": statistics.median(
                [path_length(pair["smooth"]["points"]) for pair in pairs]) if pairs else 0.0,
            "first_raw_length_m": path_length(raw[0]["points"]) if raw else 0.0,
            "first_raw_poses": len(raw[0]["points"]) if raw else 0,
            "median_raw_max_turn_deg": statistics.median(
                [pair["raw_metrics"]["max_turn_deg"] for pair in pairs]) if pairs else 0.0,
            "median_smooth_max_turn_deg": statistics.median(
                [pair["smooth_metrics"]["max_turn_deg"] for pair in pairs]) if pairs else 0.0,
            "median_raw_p95_curvature": statistics.median(
                [pair["raw_metrics"]["p95_curvature"] for pair in pairs]) if pairs else 0.0,
            "median_smooth_p95_curvature": statistics.median(
                [pair["smooth_metrics"]["p95_curvature"] for pair in pairs]) if pairs else 0.0,
            "median_raw_arc_like_run": statistics.median(
                [pair["raw_metrics"]["longest_arc_like_run"] for pair in pairs]
            ) if pairs else 0.0,
            "maximum_raw_arc_like_run": max(
                [pair["raw_metrics"]["longest_arc_like_run"] for pair in pairs],
                default=0),
            "controller_closer_to_smooth": sum(
                pair["consumed"] is not None and
                path_rms_distance(pair["consumed"]["points"], pair["smooth"]["points"]) + 0.001 <
                path_rms_distance(pair["consumed"]["points"], pair["raw"]["points"])
                for pair in pairs),
            "controller_closer_to_raw": sum(
                pair["consumed"] is not None and
                path_rms_distance(pair["consumed"]["points"], pair["raw"]["points"]) + 0.001 <
                path_rms_distance(pair["consumed"]["points"], pair["smooth"]["points"])
                for pair in pairs),
        })

    print("\nPATHS")
    for item in path_segments:
        print(json.dumps(round_values(item), ensure_ascii=False))

    print("\nGLOBAL_COUNTS")
    print(json.dumps({
        "timeline_rows": len(timeline),
        "timeline_kind_counts": Counter(row.get("kind") for row in timeline),
        "timeline_label_counts": Counter(row.get("label") for row in timeline),
        "raw_paths": len(raw_paths),
        "smoothed_paths": len(smooth_paths),
        "controller_paths": len(controller_paths),
        "approach_entries": sum("Robot to approach" in event[1] for event in journal),
        "stop_entries": sum("Robot to stop" in event[1] for event in journal),
        "smoother_collision_rejections": sum(
            "Smoothed path leads to a collision" in event[1] for event in journal),
        "planner_failures": sum(
            "failed to generate a valid path" in event[1] for event in journal),
        "control_loop_missed": sum(
            "control loop missed" in event[1].lower() for event in journal),
        "bt_tick_rate_exceeded": sum(
            "tick rate" in event[1].lower() and "exceeded" in event[1].lower()
            for event in journal),
        "current_saturation_samples": sum(
            bool(payload.get("current_saturation_mask"))
            for _, payload in wheel_diagnostics),
        "wheel_online_mask_counts": Counter(
            int(payload.get("online_mask", -1)) for _, payload in wheel_diagnostics),
    }, ensure_ascii=False, default=dict))


if __name__ == "__main__":
    main()
