import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import SpeedLimit
from std_msgs.msg import String, UInt8


class Monitor(Node):
    def __init__(self, out_dir):
        super().__init__("bed13_one_run_monitor")
        self.out_dir = out_dir
        self.latest = {}
        self.files = {}
        self.live_file = open(
            os.path.join(out_dir, "live_monitor.log"),
            "a",
            buffering=1,
            encoding="utf-8",
        )

        twist_topics = [
            ("RAW", "/cmd_vel_nav"),
            ("SMOOTH", "/cmd_vel"),
            ("HOME", "/cmd_vel_home_limited"),
            ("SAFE", "/cmd_vel_safe"),
        ]
        odom_topics = [
            ("ODOM", "/odom"),
            ("WHEEL", "/medical_nav/wheel_odom"),
        ]
        string_topics = [
            ("BRIDGE_DEBUG", "/medical_nav/bridge_cmd_debug"),
            ("BRIDGE_STATUS", "/medical_nav/bridge_status"),
            ("WHEEL_DIAG", "/medical_nav/wheel_diagnostics"),
            ("NAV_STATUS", "/medical_nav/navigator_status"),
            ("GOAL_REQUEST", "/medical_nav/goal_request"),
            ("CORNER_SPEED", "/medical_nav/corner_speed_status"),
            ("CONE_COMP", "/medical_nav/cone_compensation_status"),
        ]

        for label, topic in twist_topics:
            self.create_subscription(
                Twist,
                topic,
                lambda msg, l=label, t=topic: self.twist_cb(l, t, msg),
                100,
            )

        for label, topic in odom_topics:
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, l=label, t=topic: self.odom_cb(l, t, msg),
                100,
            )

        for label, topic in string_topics:
            self.create_subscription(
                String,
                topic,
                lambda msg, l=label, t=topic: self.string_cb(l, t, msg),
                100,
            )

        self.create_subscription(
            PoseStamped,
            "/medical_nav/robot_pose",
            lambda msg: self.pose_cb("ROBOT_POSE", "/medical_nav/robot_pose", msg),
            100,
        )

        self.create_subscription(
            PoseStamped,
            "/medical_nav/ops_raw_pose",
            lambda msg: self.pose_cb("OPS_RAW_POSE", "/medical_nav/ops_raw_pose", msg),
            100,
        )

        self.create_subscription(
            UInt8,
            "/medical_nav/task_state",
            lambda msg: self.task_cb("TASK_STATE", "/medical_nav/task_state", msg),
            100,
        )

        self.create_subscription(
            SpeedLimit,
            "/speed_limit",
            lambda msg: self.speed_cb("SPEED_LIMIT", "/speed_limit", msg),
            100,
        )

        self.create_timer(0.2, self.print_snapshot)

    def file_for(self, label):
        if label not in self.files:
            self.files[label] = open(
                os.path.join(self.out_dir, label.lower() + ".jsonl"),
                "a",
                buffering=1,
                encoding="utf-8",
            )
        return self.files[label]

    def write_record(self, label, topic, data):
        record = {
            "recv_unix_ns": time.time_ns(),
            "recv_time": datetime.now().isoformat(timespec="milliseconds"),
            "topic": topic,
            "data": data,
        }
        self.file_for(label).write(
            json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
        )

    def twist_cb(self, label, topic, msg):
        data = {
            "vx": float(msg.linear.x),
            "vy": float(msg.linear.y),
            "wz": float(msg.angular.z),
            "speed": math.hypot(float(msg.linear.x), float(msg.linear.y)),
        }
        self.latest[label] = data
        self.write_record(label, topic, data)

    def odom_cb(self, label, topic, msg):
        t = msg.twist.twist
        data = {
            "vx": float(t.linear.x),
            "vy": float(t.linear.y),
            "wz": float(t.angular.z),
            "speed": math.hypot(float(t.linear.x), float(t.linear.y)),
            "pose_x": float(msg.pose.pose.position.x),
            "pose_y": float(msg.pose.pose.position.y),
        }
        self.latest[label] = data
        self.write_record(label, topic, data)

    def pose_cb(self, label, topic, msg):
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        data = {
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "yaw_deg": math.degrees(yaw),
        }
        self.latest[label] = data
        self.write_record(label, topic, data)

    def string_cb(self, label, topic, msg):
        raw = str(msg.data)
        try:
            data = json.loads(raw)
        except Exception:
            data = {"raw": raw}
        self.latest[label] = data
        self.write_record(label, topic, data)

    def task_cb(self, label, topic, msg):
        data = {"task_state": int(msg.data)}
        self.latest[label] = data
        self.write_record(label, topic, data)

    def speed_cb(self, label, topic, msg):
        data = {
            "speed_limit": float(msg.speed_limit),
            "percentage": bool(msg.percentage),
        }
        self.latest[label] = data
        self.write_record(label, topic, data)

    @staticmethod
    def nested_find(obj, key):
        if isinstance(obj, dict):
            if key in obj:
                return obj[key]
            for value in obj.values():
                found = Monitor.nested_find(value, key)
                if found is not None:
                    return found
        elif isinstance(obj, list):
            for value in obj:
                found = Monitor.nested_find(value, key)
                if found is not None:
                    return found
        return None

    @staticmethod
    def fmt_motion(label, value):
        if not isinstance(value, dict):
            return f"{label}=--"
        return (
            f"{label}=({value.get('vx', 0.0):+.3f},"
            f"{value.get('vy', 0.0):+.3f})"
            f"|v|={value.get('speed', 0.0):.3f}"
            f" wz={value.get('wz', 0.0):+.3f}"
        )

    def print_snapshot(self):
        parts = [
            self.fmt_motion("RAW", self.latest.get("RAW")),
            self.fmt_motion("SMOOTH", self.latest.get("SMOOTH")),
            self.fmt_motion("HOME", self.latest.get("HOME")),
            self.fmt_motion("SAFE", self.latest.get("SAFE")),
            self.fmt_motion("ODOM", self.latest.get("ODOM")),
            self.fmt_motion("WHEEL", self.latest.get("WHEEL")),
        ]

        debug = self.latest.get("BRIDGE_DEBUG", {})
        sent = debug.get("sent_cmd", {}) if isinstance(debug, dict) else {}
        release = (
            debug.get("release_limited_cmd", {})
            if isinstance(debug, dict)
            else {}
        )

        diag = self.latest.get("WHEEL_DIAG", {})
        if isinstance(diag, dict) and "target_rpm" in diag:
            rpm_text = (
                f"RPMt={diag.get('target_rpm')}"
                f" RPMm={diag.get('measured_rpm')}"
                f" ERR={diag.get('rpm_error')}"
                f" SAT=0x{int(diag.get('current_saturation_mask', 0)):02x}"
            )
        else:
            rpm_text = "RPM=--"

        status = self.latest.get("BRIDGE_STATUS", {})
        yaw = self.nested_find(status, "current_yaw_deg")
        if yaw is None:
            yaw = self.nested_find(debug, "current_yaw_deg")

        task = self.latest.get("TASK_STATE", {})
        task_value = task.get("task_state") if isinstance(task, dict) else None

        line = (
            f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} | "
            + " | ".join(parts)
            + f" | RELEASE=({release.get('vx', 0.0):+.3f},"
            f"{release.get('vy', 0.0):+.3f})"
            + f" SENT=({sent.get('vx', 0.0):+.3f},"
            f"{sent.get('vy', 0.0):+.3f},"
            f"{sent.get('wz', 0.0):+.3f})"
            + f" | {rpm_text}"
            + f" | yaw={yaw if yaw is not None else '--'}"
            + f" | task={task_value if task_value is not None else '--'}"
        )

        print(line, flush=True)
        self.live_file.write(line + "\n")

    def close(self):
        for handle in self.files.values():
            handle.close()
        self.live_file.close()


def main():
    out_dir = sys.argv[1]
    rclpy.init()
    node = Monitor(out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
