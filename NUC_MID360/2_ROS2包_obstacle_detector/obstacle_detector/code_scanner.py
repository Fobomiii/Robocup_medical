#!/usr/bin/env python3
"""State-gated QR and arbitrary-angle CODE128 scanner for the DECXIN camera."""

import json
import math
import os
import subprocess
import threading
import time
from typing import Optional, Tuple

import cv2
import rclpy
import zxingcpp
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from pyzbar.pyzbar import ZBarSymbol, decode
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, UInt8

from .nav_protocol import (
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_BED3,
    SCAN_CONTEXT_ORDER,
    SCAN_FORMAT_CODE128,
    SCAN_FORMAT_QR,
)
from .field_goals import load_field_goals
from .scanner_core import (
    TASK_NAV_NURSE,
    ScanConsensus,
    expected_scan,
    scan_position_is_allowed,
    value_is_allowed,
)


ANGLE_GROUPS = (
    (-15, 15, 90),
    (-30, 30, 75),
    (-45, 45, -75),
    (-60, 60, 0),
)


def rotate_bound(image, angle):
    height, width = image.shape[:2]
    center = (width / 2.0, height / 2.0)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    cos_a = abs(matrix[0, 0])
    sin_a = abs(matrix[0, 1])
    new_width = int(height * sin_a + width * cos_a)
    new_height = int(height * cos_a + width * sin_a)
    matrix[0, 2] += new_width / 2.0 - center[0]
    matrix[1, 2] += new_height / 2.0 - center[1]
    return cv2.warpAffine(
        image,
        matrix,
        (new_width, new_height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=255,
    )


class CodeScanner(Node):
    def __init__(self) -> None:
        super().__init__("code_scanner")
        self.declare_parameter(
            "camera_device",
            "/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0",
        )
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1200)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("decode_rate_hz", 15.0)
        self.declare_parameter("confirm_hits", 2)
        self.declare_parameter("confirm_window_s", 0.8)
        self.declare_parameter("roi_left", 0.0)
        self.declare_parameter("roi_right", 1.0)
        self.declare_parameter("roi_top", 0.0)
        self.declare_parameter("roi_bottom", 1.0)
        self.declare_parameter("full_frame_fallback_period", 6)
        self.declare_parameter("bed_scan_activation_distance_m", 1.2)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("power_line_frequency", 1)
        self.declare_parameter("auto_exposure", 3)
        self.declare_parameter("autofocus", False)
        self.declare_parameter("focus_absolute", 630)
        self.declare_parameter("preview", False)
        self.declare_parameter(
            "preview_topic", "/medical_nav/scanner_preview/compressed"
        )
        self.declare_parameter("preview_rate_hz", 8.0)
        self.declare_parameter("preview_width", 640)
        self.declare_parameter("preview_height", 400)
        self.declare_parameter("preview_jpeg_quality", 75)
        default_field_config = os.path.join(
            get_package_share_directory("obstacle_detector"),
            "config",
            "field_map.yaml",
        )
        self.declare_parameter("field_config", default_field_config)

        get = lambda name: self.get_parameter(name).value
        self.camera_device = str(get("camera_device"))
        self.width = int(get("width"))
        self.height = int(get("height"))
        self.fps = float(get("fps"))
        self.roi = tuple(
            float(get(name))
            for name in ("roi_left", "roi_right", "roi_top", "roi_bottom")
        )
        if not (0.0 <= self.roi[0] < self.roi[1] <= 1.0 and
                0.0 <= self.roi[2] < self.roi[3] <= 1.0):
            raise ValueError(f"invalid scanner ROI: {self.roi}")
        self.full_frame_period = max(1, int(get("full_frame_fallback_period")))
        self.bed_scan_activation_distance_mm = max(
            0.1, float(get("bed_scan_activation_distance_m"))
        ) * 1000.0
        self.pose_timeout_s = max(0.1, float(get("pose_timeout_s")))
        self.power_line_frequency = int(get("power_line_frequency"))
        self.auto_exposure = int(get("auto_exposure"))
        self.autofocus = bool(get("autofocus"))
        self.focus_absolute = int(get("focus_absolute"))
        self.preview = bool(get("preview"))
        self.preview_width = max(160, int(get("preview_width")))
        self.preview_height = max(100, int(get("preview_height")))
        self.preview_jpeg_quality = max(
            30, min(95, int(get("preview_jpeg_quality")))
        )
        self.consensus = ScanConsensus(
            int(get("confirm_hits")), float(get("confirm_window_s"))
        )
        goals_by_name, _ = load_field_goals(str(get("field_config")))
        self.scan_targets_mm = {
            SCAN_CONTEXT_BED1: (
                goals_by_name["bed1"].x_mm,
                goals_by_name["bed1"].y_mm,
            ),
            SCAN_CONTEXT_BED3: (
                goals_by_name["bed3"].x_mm,
                goals_by_name["bed3"].y_mm,
            ),
        }

        self.task_state = 0
        self.robot_field_pose_mm = None
        self.last_pose_s = 0.0
        self.spatial_gate_open = False
        self.frame_index = 0
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.capture = None
        self.camera_online = False
        self.last_value = ""
        self.last_source = ""
        self.last_decode_ms = 0.0
        self.scan_values = {
            SCAN_CONTEXT_ORDER: "",
            SCAN_CONTEXT_BED1: "",
            SCAN_CONTEXT_BED3: "",
        }

        self.result_pub = self.create_publisher(String, "/medical_nav/scan_result", 10)
        self.status_pub = self.create_publisher(String, "/medical_nav/scanner_status", 10)
        self.preview_pub = self.create_publisher(
            CompressedImage, str(get("preview_topic")), 2
        )
        self.create_subscription(UInt8, "/medical_nav/task_state", self._task_state, 10)
        self.create_subscription(
            PoseStamped, "/medical_nav/robot_pose", self._robot_pose, 10
        )

        rate = max(1.0, float(get("decode_rate_hz")))
        preview_rate = max(1.0, float(get("preview_rate_hz")))
        self.create_timer(1.0 / rate, self._decode_tick)
        self.create_timer(1.0 / preview_rate, self._preview_tick)
        self.create_timer(1.0, self._publish_status)
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        self.get_logger().info(f"Scanner configured for {self.camera_device}")

    def _configure_v4l2(self) -> None:
        controls = [
            f"power_line_frequency={self.power_line_frequency}",
            f"auto_exposure={self.auto_exposure}",
            f"focus_automatic_continuous={1 if self.autofocus else 0}",
        ]
        if not self.autofocus:
            controls.append(f"focus_absolute={self.focus_absolute}")
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", self.camera_device, "--set-ctrl", ",".join(controls)],
                check=False,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.get_logger().warn(f"Cannot apply camera controls: {exc}")
            return
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip()
            self.get_logger().warn(f"Camera controls were not fully applied: {detail}")

    def _open_camera(self):
        self._configure_v4l2()
        capture = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(cv2.CAP_PROP_AUTOFOCUS, 1 if self.autofocus else 0)
        if not self.autofocus:
            capture.set(cv2.CAP_PROP_FOCUS, self.focus_absolute)
        if not capture.isOpened():
            capture.release()
            return None
        return capture

    def _capture_loop(self) -> None:
        while not self.stop_event.is_set():
            if self.capture is None:
                self.capture = self._open_camera()
                self.camera_online = self.capture is not None
                if self.capture is None:
                    time.sleep(2.0)
                    continue
            ok, frame = self.capture.read()
            if not ok:
                self.camera_online = False
                self.capture.release()
                self.capture = None
                time.sleep(0.2)
                continue
            with self.frame_lock:
                self.latest_frame = frame

    def _preview_tick(self) -> None:
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return

        preview = cv2.resize(
            frame,
            (self.preview_width, self.preview_height),
            interpolation=cv2.INTER_AREA,
        )
        height, width = preview.shape[:2]
        left, right, top, bottom = self.roi
        cv2.rectangle(
            preview,
            (int(width * left), int(height * top)),
            (
                min(width - 1, int(width * right)),
                min(height - 1, int(height * bottom)),
            ),
            (0, 220, 255),
            2,
        )
        ok, encoded = cv2.imencode(
            ".jpg",
            preview,
            [cv2.IMWRITE_JPEG_QUALITY, self.preview_jpeg_quality],
        )
        if not ok:
            return

        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.preview_pub.publish(message)

    def _task_state(self, message: UInt8) -> None:
        state = int(message.data)
        if state != self.task_state:
            if state == TASK_NAV_NURSE:
                self.scan_values = dict.fromkeys(self.scan_values, "")
            self.task_state = state
            self.consensus.reset()
            self.spatial_gate_open = False
            self.last_value = ""
            self.last_source = ""

    def _robot_pose(self, message: PoseStamped) -> None:
        # ROS map (x forward, y left) -> surveyed field (x right, y forward).
        self.robot_field_pose_mm = (
            -float(message.pose.position.y) * 1000.0,
            float(message.pose.position.x) * 1000.0,
        )
        self.last_pose_s = time.monotonic()

    def _target_distance_m(self, context: int) -> Optional[float]:
        if context == SCAN_CONTEXT_ORDER:
            return 0.0
        if (
            self.robot_field_pose_mm is None
            or time.monotonic() - self.last_pose_s > self.pose_timeout_s
        ):
            return None
        target = self.scan_targets_mm.get(context)
        if target is None:
            return None
        return math.hypot(
            self.robot_field_pose_mm[0] - target[0],
            self.robot_field_pose_mm[1] - target[1],
        ) / 1000.0

    def _scan_position_is_allowed(self, context: int) -> bool:
        if context == SCAN_CONTEXT_ORDER:
            return True
        if (
            self.robot_field_pose_mm is None
            or time.monotonic() - self.last_pose_s > self.pose_timeout_s
        ):
            return False
        return scan_position_is_allowed(
            context,
            self.robot_field_pose_mm[0],
            self.robot_field_pose_mm[1],
            self.scan_targets_mm,
            self.bed_scan_activation_distance_mm,
        )

    @staticmethod
    def _decode_qr(gray) -> Tuple[str, str]:
        results = zxingcpp.read_barcodes(
            gray,
            formats=zxingcpp.BarcodeFormat.QRCode,
            try_rotate=True,
            try_downscale=True,
            try_invert=True,
        )
        for result in results:
            value = result.text.strip()
            if value_is_allowed(SCAN_FORMAT_QR, value):
                return value, "zxing_qr"
        return "", ""

    def _decode_barcode(self, gray) -> Tuple[str, str]:
        images = [(gray, 0)]
        for angle in ANGLE_GROUPS[self.frame_index % len(ANGLE_GROUPS)]:
            if angle != 0:
                images.append((rotate_bound(gray, angle), angle))
        for image, angle in images:
            for result in decode(image, symbols=[ZBarSymbol.CODE128]):
                value = result.data.decode("ascii", "ignore").strip()
                if value_is_allowed(SCAN_FORMAT_CODE128, value):
                    return value, f"zbar_{angle}"
        if self.frame_index % 4 == 0:
            results = zxingcpp.read_barcodes(
                gray,
                formats=zxingcpp.BarcodeFormat.Code128,
                try_rotate=True,
                try_downscale=True,
                try_invert=False,
            )
            for result in results:
                value = result.text.strip()
                if value_is_allowed(SCAN_FORMAT_CODE128, value):
                    return value, "zxing_code128"
        return "", ""

    def _decode(self, gray, format: int) -> Tuple[str, str]:
        return self._decode_qr(gray) if format == SCAN_FORMAT_QR else self._decode_barcode(gray)

    def _decode_tick(self) -> None:
        expected = expected_scan(self.task_state)
        if expected is None:
            return
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return

        context, format = expected
        spatial_gate_open = self._scan_position_is_allowed(context)
        if spatial_gate_open != self.spatial_gate_open:
            self.consensus.reset()
            self.spatial_gate_open = spatial_gate_open
        if not spatial_gate_open:
            return

        height, width = frame.shape[:2]
        left, right, top, bottom = self.roi
        x1, x2 = int(width * left), int(width * right)
        y1, y2 = int(height * top), int(height * bottom)
        full_frame = x1 == 0 and x2 == width and y1 == 0 and y2 == height
        scan_frame = frame if full_frame else frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(scan_frame, cv2.COLOR_BGR2GRAY)

        started = time.perf_counter()
        value, source = self._decode(gray, format)
        if (
            not value
            and not full_frame
            and self.frame_index % self.full_frame_period == 0
        ):
            full_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            value, source = self._decode(full_gray, format)
            if value:
                source += "_full"
        self.last_decode_ms = (time.perf_counter() - started) * 1000.0
        self.frame_index += 1

        if value and self.consensus.observe(format, value, time.monotonic()):
            self.last_value = value
            self.last_source = source
            self.scan_values[context] = value
            message = String()
            message.data = json.dumps(
                {"context": context, "format": format, "value": value, "source": source}
            )
            self.result_pub.publish(message)
            self.get_logger().info(
                f"Confirmed scan context={context} format={format} value={value} via {source}"
            )

        if self.preview:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 255), 3)
            cv2.putText(
                frame,
                f"state={self.task_state} value={self.last_value}",
                (25, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                3,
            )
            cv2.imshow("Medical code scanner", cv2.resize(frame, (960, 600)))
            cv2.waitKey(1)

    def _publish_status(self) -> None:
        expected = expected_scan(self.task_state)
        context = expected[0] if expected else 0
        message = String()
        message.data = json.dumps(
            {
                "camera": self.camera_online,
                "device": self.camera_device,
                "task_state": self.task_state,
                "active": expected is not None,
                "context": context,
                "format": expected[1] if expected else 0,
                "value": self.last_value,
                "source": self.last_source,
                "decode_ms": round(self.last_decode_ms, 1),
                "scan_values": self.scan_values,
                "spatial_gate_open": self._scan_position_is_allowed(context)
                if expected
                else False,
                "target_distance_m": self._target_distance_m(context)
                if expected
                else None,
            }
        )
        self.status_pub.publish(message)

    def destroy_node(self):
        self.stop_event.set()
        self.capture_thread.join(timeout=2.0)
        if self.capture is not None:
            self.capture.release()
        if self.preview:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CodeScanner()
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
