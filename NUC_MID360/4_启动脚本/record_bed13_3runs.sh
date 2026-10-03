#!/usr/bin/env bash
set -Eeo pipefail

# ROS setup scripts are not compatible with nounset in Humble.
set +u
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
set -u

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/bed13_full_$STAMP"
ARCHIVE="$OUT.tar.gz"
START_TIME="$(date '+%Y-%m-%d %H:%M:%S')"
JOURNAL_PID=""
SYSTEM_PID=""
BAG_PID=""
CLEANED_UP=0

mkdir -p "$OUT"

BASE_BAG_TOPICS=(
    /cmd_vel_nav
    /cmd_vel
    /cmd_vel_home_limited
    /cmd_vel_safe
    /medical_nav/task_state
    /medical_nav/navigator_status
    /medical_nav/bed_approach_status
    /medical_nav/cone_compensation_status
    /medical_nav/blind_zone_status
    /medical_nav/lidar_filter_status
    /medical_nav/lidar_safety_filter_status
    /medical_nav/robot_pose
    /controller_selector
    /medical_nav/bridge_cmd_debug
    /medical_nav/bridge_status
    /medical_nav/stm32_ready
    /medical_nav/corner_speed_status
    /medical_nav/execution_path
    /medical_nav/execution_path_status
    /medical_nav/wheel_odom
    /medical_nav/wheel_diagnostics
    /odom
    /plan
    /plan_smoothed
    /transformed_global_plan
    /speed_limit
    /navigate_to_pose/_action/status
    /tf
    /tf_static
)

snapshot() {
    {
        echo "========== TIME =========="
        date --iso-8601=ns
        echo
        echo "========== TOPICS =========="
        ros2 topic list -t 2>&1 || true
        echo
        echo "========== NODES =========="
        ros2 node list 2>&1 || true
        echo
        echo "========== SAFETY POINT CLOUD =========="
        timeout 5s ros2 topic info -v /livox/lidar_safety 2>&1 || true
        timeout 5s ros2 topic echo --once /medical_nav/lidar_safety_filter_status 2>&1 || true
        echo
        echo "========== PLANNING POINT CLOUD =========="
        timeout 5s ros2 topic info -v /livox/lidar_nav 2>&1 || true
        timeout 5s ros2 topic echo --once /medical_nav/lidar_filter_status 2>&1 || true
        echo
        echo "========== BRIDGE COMMAND =========="
        timeout 5s ros2 topic echo --once /medical_nav/bridge_cmd_debug 2>&1 || true
        echo
        echo "========== BED APPROACH ENVELOPE =========="
        timeout 5s ros2 topic echo --once /medical_nav/bed_approach_status 2>&1 || true
        echo
        echo "========== COLLISION MONITOR =========="
        timeout 8s ros2 param dump /collision_monitor 2>&1 || true
        echo
        echo "========== STM32 BRIDGE =========="
        timeout 8s ros2 param dump /stm32_bridge 2>&1 || true
        echo
        echo "========== CONTROLLER =========="
        timeout 8s ros2 param dump /controller_server 2>&1 || true
        echo
        echo "========== LIFECYCLE =========="
        for node in \
            /controller_server \
            /planner_server \
            /smoother_server \
            /behavior_server \
            /bt_navigator \
            /velocity_smoother \
            /collision_monitor
        do
            printf '\n--- %s ---\n' "$node"
            timeout 5s ros2 lifecycle get "$node" 2>&1 || true
        done
    } >"$OUT/snapshot.txt" 2>&1
}

stop_process() {
    local pid="${1:-}"
    local signal="${2:-TERM}"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        kill -"$signal" "$pid" 2>/dev/null || true
    fi
}

cleanup() {
    if [[ "$CLEANED_UP" -eq 1 ]]; then
        return
    fi
    CLEANED_UP=1
    trap - EXIT INT TERM

    echo
    echo "Stopping recorder and finalizing rosbag..."
    stop_process "$BAG_PID" INT
    if [[ -n "$BAG_PID" ]]; then
        for _ in $(seq 1 100); do
            kill -0 "$BAG_PID" 2>/dev/null || break
            sleep 0.1
        done
        stop_process "$BAG_PID" TERM
        wait "$BAG_PID" 2>/dev/null || true
    fi
    stop_process "$JOURNAL_PID" TERM
    stop_process "$SYSTEM_PID" TERM
    wait "$JOURNAL_PID" 2>/dev/null || true
    wait "$SYSTEM_PID" 2>/dev/null || true

    snapshot
    tar -C "$HOME" -czf "$ARCHIVE" "$(basename "$OUT")"

    echo
    echo "Recording complete:"
    echo "$ARCHIVE"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

journalctl \
    -u obstacle-detector.service \
    --since "$START_TIME" \
    -f \
    -o short-precise \
    >"$OUT/journal.log" 2>&1 &
JOURNAL_PID=$!

(
    while true; do
        echo "========== $(date --iso-8601=ns) =========="
        uptime
        free -m
        ps -eo pid,ppid,stat,pcpu,pmem,nlwp,comm,args \
            --sort=-pcpu | head -n 35
        echo
        sleep 1
    done
) >"$OUT/system.log" 2>&1 &
SYSTEM_PID=$!

mapfile -t COLLISION_BAG_TOPICS < <(
    timeout 8s ros2 topic list 2>/dev/null \
        | grep -Ei 'collision|polygon|StopPolygon|ApproachPolygon' \
        | sort -u \
        || true
)
BAG_TOPICS=("${BASE_BAG_TOPICS[@]}")
for topic in "${COLLISION_BAG_TOPICS[@]}"; do
    BAG_TOPICS+=("$topic")
done
printf '%s\n' "${BAG_TOPICS[@]}" >"$OUT/recorded_topics.txt"

ros2 bag record \
    --storage sqlite3 \
    -o "$OUT/rosbag" \
    "${BAG_TOPICS[@]}" \
    >"$OUT/rosbag.log" 2>&1 &
BAG_PID=$!

sleep 1
if ! kill -0 "$BAG_PID" 2>/dev/null; then
    echo "rosbag recorder failed to start:"
    cat "$OUT/rosbag.log"
    exit 2
fi

/usr/bin/python3 - "$OUT/timeline.jsonl" <<'PY'
import json
import math
import sys
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String, UInt8


OUTPUT_FILE = sys.argv[1]
TWIST_TOPICS = {
    "RAW": "/cmd_vel_nav",
    "SMOOTH": "/cmd_vel",
    "HOME": "/cmd_vel_home_limited",
    "SAFE": "/cmd_vel_safe",
}
STRING_TOPICS = {
    "NAV_STATUS": "/medical_nav/navigator_status",
    "BED_APPROACH": "/medical_nav/bed_approach_status",
    "CONE": "/medical_nav/cone_compensation_status",
    "BLIND": "/medical_nav/blind_zone_status",
    "BRIDGE_CMD": "/medical_nav/bridge_cmd_debug",
    "BRIDGE_STATUS": "/medical_nav/bridge_status",
    "CORNER": "/medical_nav/corner_speed_status",
    "EXECUTION_PATH": "/medical_nav/execution_path_status",
    "WHEEL_DIAG": "/medical_nav/wheel_diagnostics",
    "LIDAR_NAV_FILTER": "/medical_nav/lidar_filter_status",
    "LIDAR_SAFETY_FILTER": "/medical_nav/lidar_safety_filter_status",
}
CLOUD_TOPICS = (
    "/livox/lidar_nav",
    "/livox/lidar_safety",
)
REQUIRED_PUBLISHERS = (
    "/cmd_vel_nav",
    "/cmd_vel",
    "/cmd_vel_safe",
    "/medical_nav/task_state",
    "/medical_nav/navigator_status",
    "/medical_nav/bed_approach_status",
    "/medical_nav/bridge_cmd_debug",
    "/medical_nav/stm32_ready",
    "/medical_nav/wheel_odom",
    "/odom",
    "/livox/lidar_nav",
    "/livox/lidar_safety",
    "/medical_nav/lidar_safety_filter_status",
)


class Bed13Recorder(Node):
    def __init__(self):
        super().__init__("bed13_full_recorder")
        self.start_mono = time.monotonic()
        self.output = open(OUTPUT_FILE, "w", buffering=1, encoding="utf-8")
        self.task_state = None
        self.last_console = 0.0
        self.last_speed = {name: None for name in TWIST_TOPICS}
        self.cloud_age = {topic: None for topic in CLOUD_TOPICS}
        self.cloud_last_rx = {topic: None for topic in CLOUD_TOPICS}
        self.cloud_max_gap = {topic: 0.0 for topic in CLOUD_TOPICS}
        self.cloud_gap_count = {topic: 0 for topic in CLOUD_TOPICS}
        self.sent_speed = None
        self.wheel_speed = None
        self.subscriptions = []

        normal_qos = QoSProfile(depth=100)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        for label, topic in TWIST_TOPICS.items():
            self.subscriptions.append(
                self.create_subscription(
                    Twist, topic, self.make_twist_callback(label, topic), normal_qos
                )
            )
        for label, topic in STRING_TOPICS.items():
            self.subscriptions.append(
                self.create_subscription(
                    String, topic, self.make_string_callback(label, topic), normal_qos
                )
            )
        for topic in CLOUD_TOPICS:
            self.subscriptions.append(
                self.create_subscription(
                    PointCloud2, topic, self.make_cloud_callback(topic), sensor_qos
                )
            )
        self.subscriptions.append(
            self.create_subscription(
                UInt8, "/medical_nav/task_state", self.task_callback, normal_qos
            )
        )
        self.subscriptions.append(
            self.create_subscription(
                Bool,
                "/medical_nav/stm32_ready",
                lambda message: self.emit("stm32_ready", ready=bool(message.data)),
                normal_qos,
            )
        )
        for label, topic in (
            ("ODOM", "/odom"),
            ("WHEEL_ODOM", "/medical_nav/wheel_odom"),
        ):
            self.subscriptions.append(
                self.create_subscription(
                    Odometry,
                    topic,
                    self.make_odom_callback(label, topic),
                    sensor_qos,
                )
            )
        self.create_timer(0.5, self.console_callback)
        self.emit("recorder_start", required_publishers=REQUIRED_PUBLISHERS)

    def emit(self, kind, **fields):
        record = {
            "wall_ns": time.time_ns(),
            "mono_s": round(time.monotonic() - self.start_mono, 6),
            "kind": kind,
            "task_state": self.task_state,
        }
        record.update(fields)
        self.output.write(
            json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
        )

    def make_twist_callback(self, label, topic):
        def callback(message):
            vx = float(message.linear.x)
            vy = float(message.linear.y)
            wz = float(message.angular.z)
            speed = math.hypot(vx, vy)
            self.last_speed[label] = speed
            self.emit(
                "twist",
                layer=label,
                topic=topic,
                vx=round(vx, 6),
                vy=round(vy, 6),
                wz=round(wz, 6),
                speed=round(speed, 6),
            )

        return callback

    def make_string_callback(self, label, topic):
        def callback(message):
            self.emit("string", label=label, topic=topic, data=message.data)
            if label == "BRIDGE_CMD":
                try:
                    payload = json.loads(message.data)
                    sent = payload.get("sent_cmd") or {}
                    self.sent_speed = float(sent.get("speed", 0.0))
                except (TypeError, ValueError, json.JSONDecodeError):
                    pass

        return callback

    def make_cloud_callback(self, topic):
        def callback(message):
            now_mono = time.monotonic()
            previous_rx = self.cloud_last_rx[topic]
            receive_gap_s = (
                None if previous_rx is None else now_mono - previous_rx
            )
            self.cloud_last_rx[topic] = now_mono
            if receive_gap_s is not None:
                self.cloud_max_gap[topic] = max(
                    self.cloud_max_gap[topic], receive_gap_s
                )
                if receive_gap_s > 0.20:
                    self.cloud_gap_count[topic] += 1
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e9
            self.cloud_age[topic] = age_s
            self.emit(
                "pointcloud_header",
                topic=topic,
                frame_id=message.header.frame_id,
                stamp_ns=stamp_ns,
                age_s=round(age_s, 6),
                receive_gap_s=(
                    None if receive_gap_s is None else round(receive_gap_s, 6)
                ),
                width=int(message.width),
                height=int(message.height),
                point_step=int(message.point_step),
            )

        return callback

    def make_odom_callback(self, label, topic):
        def callback(message):
            vx = float(message.twist.twist.linear.x)
            vy = float(message.twist.twist.linear.y)
            wz = float(message.twist.twist.angular.z)
            speed = math.hypot(vx, vy)
            if label == "WHEEL_ODOM":
                self.wheel_speed = speed
            self.emit(
                "odometry",
                label=label,
                topic=topic,
                vx=round(vx, 6),
                vy=round(vy, 6),
                wz=round(wz, 6),
                speed=round(speed, 6),
            )

        return callback

    def task_callback(self, message):
        current = int(message.data)
        if current != self.task_state:
            previous = self.task_state
            self.task_state = current
            self.emit("task_state", previous=previous, current=current)
            print(f"\n[TASK] {previous} -> {current}", flush=True)

    def console_callback(self):
        now = time.monotonic()
        if now - self.last_console < 0.45:
            return
        self.last_console = now
        speeds = " ".join(
            f"{label}={'--' if speed is None else f'{speed:.3f}'}"
            for label, speed in self.last_speed.items()
        )
        nav_age = self.cloud_age["/livox/lidar_nav"]
        safety_age = self.cloud_age["/livox/lidar_safety"]
        nav_text = "--" if nav_age is None else f"{nav_age:.3f}s"
        safety_text = "--" if safety_age is None else f"{safety_age:.3f}s"
        sent_text = "--" if self.sent_speed is None else f"{self.sent_speed:.3f}"
        wheel_text = "--" if self.wheel_speed is None else f"{self.wheel_speed:.3f}"
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(
            f"{stamp} TASK={self.task_state} {speeds} SENT={sent_text} "
            f"WHEEL={wheel_text} CLOUD_NAV={nav_text} CLOUD_SAFETY={safety_text}",
            flush=True,
        )

    def close(self):
        self.emit(
            "recorder_stop",
            cloud_gap_count=self.cloud_gap_count,
            cloud_max_gap_s={
                topic: round(value, 6)
                for topic, value in self.cloud_max_gap.items()
            },
        )
        self.output.close()


rclpy.init()
node = Bed13Recorder()

try:
    last_wait_print = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        missing = [
            topic
            for topic in REQUIRED_PUBLISHERS
            if node.count_publishers(topic) == 0
        ]
        now = time.monotonic()
        if not missing:
            node.emit("recorder_ready", publishers=REQUIRED_PUBLISHERS)
            print()
            print("============================================================")
            print("RECORDER READY")
            print("Run three complete missions, then wait two seconds and press Ctrl+C.")
            print("============================================================")
            print()
            break
        if now - last_wait_print >= 2.0:
            last_wait_print = now
            print("Waiting for publishers: " + ", ".join(missing), flush=True)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
except KeyboardInterrupt:
    pass
finally:
    node.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
