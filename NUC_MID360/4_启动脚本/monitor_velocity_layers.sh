#!/usr/bin/env bash
set -euo pipefail

RATE_HZ="${1:-5.0}"
STALE_S="${2:-0.75}"

set +u
if [ -f "$HOME/.config/medical-navigation.env" ]; then
    set -a
    # shellcheck disable=SC1091
    source "$HOME/.config/medical-navigation.env"
    set +a
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/livox_ws/install/setup.bash"
set -u

exec /usr/bin/python3 - "$RATE_HZ" "$STALE_S" <<'PY'
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class VelocityLayerMonitor(Node):
    TWIST_TOPICS = (
        ("RAW", "/cmd_vel_nav"),
        ("SMOOTH", "/cmd_vel"),
        ("HOME", "/cmd_vel_home_limited"),
        ("SAFE", "/cmd_vel_safe"),
    )
    ODOM_TOPICS = (
        ("ODOM", "/odom"),
        ("WHEEL", "/medical_nav/wheel_odom"),
    )

    def __init__(self, rate_hz: float, stale_s: float) -> None:
        super().__init__(f"velocity_layer_monitor_{os.getpid()}")
        self.rate_hz = max(0.5, float(rate_hz))
        self.stale_s = max(0.2, float(stale_s))
        self.samples = {}
        self.previous_output = {}
        self.speed_limit = None
        self.speed_limit_seen = 0.0

        for label, topic in self.TWIST_TOPICS:
            self.create_subscription(
                Twist,
                topic,
                lambda message, name=label: self._twist(name, message),
                20,
            )
        for label, topic in self.ODOM_TOPICS:
            self.create_subscription(
                Odometry,
                topic,
                lambda message, name=label: self._odom(name, message),
                20,
            )
        self.create_subscription(SpeedLimit, "/speed_limit", self._limit, 10)
        self.publisher = self.create_publisher(
            String, "/medical_nav/velocity_layers", 10
        )
        self.create_timer(1.0 / self.rate_hz, self._publish)

        topic_text = " ".join(
            f"{label}={topic}" for label, topic in self.TWIST_TOPICS + self.ODOM_TOPICS
        )
        print(
            f"Velocity layer monitor started at {self.rate_hz:.2f} Hz\n"
            f"{topic_text} LIMIT=/speed_limit\n"
            "Format: LABEL=(vx,vy) |v|=planar_speed wz=yaw_rate "
            "a=planar_speed_change_rate",
            flush=True,
        )

    def _store(self, label: str, vx: float, vy: float, wz: float) -> None:
        self.samples[label] = {
            "vx": float(vx),
            "vy": float(vy),
            "wz": float(wz),
            "seen": time.monotonic(),
        }

    def _twist(self, label: str, message: Twist) -> None:
        self._store(label, message.linear.x, message.linear.y, message.angular.z)

    def _odom(self, label: str, message: Odometry) -> None:
        twist = message.twist.twist
        self._store(label, twist.linear.x, twist.linear.y, twist.angular.z)

    def _limit(self, message: SpeedLimit) -> None:
        self.speed_limit = {
            "value": float(message.speed_limit),
            "percentage": bool(message.percentage),
        }
        self.speed_limit_seen = time.monotonic()

    def _layer_snapshot(self, label: str, now: float):
        sample = self.samples.get(label)
        if sample is None or now - sample["seen"] > self.stale_s:
            self.previous_output.pop(label, None)
            return None

        speed = math.hypot(sample["vx"], sample["vy"])
        acceleration = None
        previous = self.previous_output.get(label)
        if previous is not None:
            elapsed = now - previous["time"]
            if elapsed > 1.0e-6:
                acceleration = (speed - previous["speed"]) / elapsed
        self.previous_output[label] = {"time": now, "speed": speed}
        return {
            "vx_m_s": sample["vx"],
            "vy_m_s": sample["vy"],
            "speed_m_s": speed,
            "wz_rad_s": sample["wz"],
            "accel_m_s2": acceleration,
            "age_s": now - sample["seen"],
        }

    @staticmethod
    def _format_layer(label: str, sample) -> str:
        if sample is None:
            return f"{label}=--"
        acceleration = sample["accel_m_s2"]
        accel_text = "--" if acceleration is None else f"{acceleration:+.2f}"
        return (
            f"{label}=({sample['vx_m_s']:+.3f},{sample['vy_m_s']:+.3f}) "
            f"|v|={sample['speed_m_s']:.3f} wz={sample['wz_rad_s']:+.3f} "
            f"a={accel_text}"
        )

    def _publish(self) -> None:
        now = time.monotonic()
        ordered_labels = [
            label for label, _ in self.TWIST_TOPICS + self.ODOM_TOPICS
        ]
        layers = {
            label: self._layer_snapshot(label, now) for label in ordered_labels
        }
        limit = None
        if (
            self.speed_limit is not None
            and now - self.speed_limit_seen <= self.stale_s
        ):
            limit = dict(self.speed_limit)
            limit["age_s"] = now - self.speed_limit_seen

        payload = {
            "rate_hz": self.rate_hz,
            "monotonic_s": now,
            "layers": layers,
            "speed_limit": limit,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(message)

        timestamp = time.strftime("%H:%M:%S")
        milliseconds = int((time.time() % 1.0) * 1000.0)
        parts = [self._format_layer(label, layers[label]) for label in ordered_labels]
        if limit is None:
            parts.append("LIMIT=--")
        elif limit["percentage"]:
            parts.append(f"LIMIT={limit['value']:.1f}%")
        else:
            parts.append(f"LIMIT={limit['value']:.3f}m/s")
        print(f"{timestamp}.{milliseconds:03d} | " + " | ".join(parts), flush=True)


def main() -> None:
    try:
        rate_hz = float(sys.argv[1])
        stale_s = float(sys.argv[2])
    except (IndexError, ValueError) as error:
        raise SystemExit(
            "Usage: monitor_velocity_layers.sh [rate_hz] [stale_timeout_s]: "
            f"{error}"
        )

    rclpy.init()
    node = VelocityLayerMonitor(rate_hz, stale_s)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
