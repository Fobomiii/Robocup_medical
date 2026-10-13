#!/usr/bin/env python3
"""Single read-only startup probe for the medical navigation stack."""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _valid_camera(status, prefix=""):
    online_key = f"{prefix}camera"
    profile_key = f"{prefix}camera_profile"
    resolution_key = f"{prefix}camera_resolution"
    resolution = status.get(resolution_key)
    return (
        status.get(online_key) is True
        and isinstance(status.get(profile_key), str)
        and status.get(profile_key) not in {"", "unavailable"}
        and isinstance(resolution, list)
        and len(resolution) == 2
        and all(isinstance(value, (int, float)) and value > 0 for value in resolution)
    )


def readiness_reason(statuses, require_hardware=False, require_scanner=False,
                     require_health=False):
    navigator = statuses.get("navigator")
    if navigator is None:
        return "navigator_status_missing"
    if navigator.get("startup_ready") is not True:
        inactive = navigator.get("inactive_startup_nodes") or []
        if inactive:
            return "nav2_inactive:" + ",".join(str(name) for name in inactive)
        actions = navigator.get("startup_actions") or {}
        missing = [name for name, ready in actions.items() if not ready]
        return "nav2_actions_missing:" + ",".join(missing or ["unknown"])

    if require_hardware:
        bridge = statuses.get("bridge")
        if bridge is None:
            return "bridge_status_missing"
        for key in ("serial", "pose", "wheel_telemetry", "stm32_ready"):
            if bridge.get(key) is not True:
                return f"bridge_{key}_not_ready"
        wheel = bridge.get("wheel") or {}
        if int(wheel.get("online_mask", 0)) != 0x0F:
            return f"wheel_online_mask_{int(wheel.get('online_mask', 0))}"

    if require_scanner:
        scanner = statuses.get("scanner")
        if scanner is None:
            return "scanner_status_missing"
        if not _valid_camera(scanner):
            return "scanner_primary_not_ready"
        if not _valid_camera(scanner, "tele_"):
            return "scanner_tele_not_ready"

    if require_health and statuses.get("health") is None:
        return "health_status_missing"
    return "ready"


class StartupProbe(Node):
    def __init__(self):
        super().__init__("medical_startup_probe")
        self.statuses = {}
        for name, topic in (
            ("navigator", "/medical_nav/navigator_status"),
            ("bridge", "/medical_nav/bridge_status"),
            ("scanner", "/medical_nav/scanner_status"),
            ("health", "/medical_nav/health_status"),
        ):
            self.create_subscription(
                String,
                topic,
                lambda message, key=name: self._status(key, message),
                10,
            )

    def _status(self, name, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self.statuses[name] = payload


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=75.0)
    parser.add_argument("--stable-samples", type=int, default=2)
    parser.add_argument("--require-hardware", action="store_true")
    parser.add_argument("--require-scanner", action="store_true")
    parser.add_argument("--require-health", action="store_true")
    return parser.parse_args(argv)


def main(args=None):
    options = parse_args(args)
    rclpy.init(args=[])
    node = StartupProbe()
    deadline = time.monotonic() + max(0.1, options.timeout)
    stable = 0
    last_reason = None
    reason = "not_checked"
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
            reason = readiness_reason(
                node.statuses,
                require_hardware=options.require_hardware,
                require_scanner=options.require_scanner,
                require_health=options.require_health,
            )
            if reason == "ready":
                stable += 1
                if stable >= max(1, options.stable_samples):
                    print(json.dumps({"ready": True, "statuses": node.statuses},
                                     ensure_ascii=False, separators=(",", ":")))
                    return 0
            else:
                stable = 0
            if reason != last_reason:
                print(f"[startup-probe] waiting: {reason}", flush=True)
                last_reason = reason
        print(json.dumps({"ready": False, "reason": reason,
                          "statuses": node.statuses},
                         ensure_ascii=False, separators=(",", ":")),
              file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
