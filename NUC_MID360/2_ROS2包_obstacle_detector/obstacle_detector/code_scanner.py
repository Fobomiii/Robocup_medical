#!/usr/bin/env python3
"""State-gated QR and CODE128 scanner with a pre-start telephoto assist."""

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
    TASK_WAIT_START,
    ScanConsensus,
    expected_scan,
    scan_position_is_allowed,
    tele_camera_assists,
    tele_camera_should_capture,
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
        self.declare_parameter(
            "tele_camera_device",
            "/dev/v4l/by-id/usb-BLC-240823--A_SDYH-8P0P-video-index0",
        )
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1200)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("tele_width", 1920)
        self.declare_parameter("tele_height", 1080)
        self.declare_parameter("tele_fps", 5.0)
        self.declare_parameter("tele_scan_timeout_s", 15.0)
        self.declare_parameter("decode_rate_hz", 15.0)
        self.declare_parameter("confirm_hits", 2)
        self.declare_parameter("confirm_window_s", 0.8)
        self.declare_parameter("decode_width", 1280)
        self.declare_parameter("decode_height", 800)
        self.declare_parameter("full_resolution_fallback_period", 4)
        self.declare_parameter("roi_left", 0.0)
        self.declare_parameter("roi_right", 1.0)
        self.declare_parameter("roi_top", 0.0)
        self.declare_parameter("roi_bottom", 1.0)
        self.declare_parameter("full_frame_fallback_period", 6)
        self.declare_parameter("bed_scan_activation_distance_m", 1.5)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("power_line_frequency", 1)
        self.declare_parameter("auto_exposure", 3)
        self.declare_parameter("autofocus", False)
        self.declare_parameter("focus_absolute", 630)
        self.declare_parameter("preview", False)
        self.declare_parameter(
            "preview_topic", "/medical_nav/scanner_preview/compressed"
        )
        self.declare_parameter(
            "tele_preview_topic", "/medical_nav/tele_scanner_preview/compressed"
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
        self.tele_camera_device = str(get("tele_camera_device"))
        self.width = int(get("width"))
        self.height = int(get("height"))
        self.fps = float(get("fps"))
        self.tele_width = int(get("tele_width"))
        self.tele_height = int(get("tele_height"))
        self.tele_fps = float(get("tele_fps"))
        self.tele_scan_timeout_s = max(1.0, float(get("tele_scan_timeout_s")))
        self.decode_width = max(320, int(get("decode_width")))
        self.decode_height = max(200, int(get("decode_height")))
        self.full_resolution_fallback_period = max(
            1, int(get("full_resolution_fallback_period"))
        )
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
        self.latest_tele_frame = None
        self.frame_lock = threading.Lock()
        self.tele_frame_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.capture = None
        self.tele_capture = None
        self.camera_online = False
        self.tele_camera_online = False
        self.camera_profile = "unavailable"
        self.tele_camera_profile = "unavailable"
        self.camera_resolution = [0, 0]
        self.tele_camera_resolution = [0, 0]
        self.tele_scan_started_s = 0.0
        self.tele_scan_timed_out = False
        self.tele_scan_succeeded = False
        self.mission_epoch = None
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
        self.tele_preview_pub = self.create_publisher(
            CompressedImage, str(get("tele_preview_topic")), 2
        )
        self.create_subscription(UInt8, "/medical_nav/task_state", self._task_state, 10)
        self.create_subscription(
            String, "/medical_nav/bridge_status", self._bridge_status, 10
        )
        self.create_subscription(
            PoseStamped, "/medical_nav/robot_pose", self._robot_pose, 10
        )

        rate = max(1.0, float(get("decode_rate_hz")))
        preview_rate = max(1.0, float(get("preview_rate_hz")))
        self.create_timer(1.0 / rate, self._decode_tick)
        self.create_timer(1.0 / preview_rate, self._preview_tick)
        self.create_timer(1.0, self._publish_status)
        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            args=(False,),
            daemon=True,
        )
        self.tele_capture_thread = threading.Thread(
            target=self._capture_loop,
            args=(True,),
            daemon=True,
        )
        self.capture_thread.start()
        self.tele_capture_thread.start()
        self.get_logger().info(
            f"Scanner configured for primary={self.camera_device} "
            f"tele={self.tele_camera_device}"
        )

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

    def _open_camera(self, tele: bool = False):
        device = self.tele_camera_device if tele else self.camera_device
        width = self.tele_width if tele else self.width
        height = self.tele_height if tele else self.height
        fps = self.tele_fps if tele else self.fps
        if not tele:
            self._configure_v4l2()
        profiles = (
            (("YUYV", True), ("driver-default", False))
            if tele
            else (("MJPG", True), ("driver-default", False))
        )
        for profile_name, configure_format in profiles:
            self.get_logger().info(
                f"Opening {'tele' if tele else 'primary'} camera {device} "
                f"with {profile_name}:{width}x{height}@{fps:g}"
            )
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not capture.isOpened():
                capture.release()
                continue
            if not tele:
                capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if configure_format:
                capture.set(
                    cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc(*profile_name),
                )
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                capture.set(cv2.CAP_PROP_FPS, fps)
                if not tele:
                    capture.set(cv2.CAP_PROP_AUTOFOCUS, 1 if self.autofocus else 0)
                    if not self.autofocus:
                        capture.set(cv2.CAP_PROP_FOCUS, self.focus_absolute)

            first_frame = None
            for _ in range(5):
                ok, candidate = capture.read()
                if ok and candidate is not None and candidate.size:
                    first_frame = candidate
                    break
            if first_frame is None:
                self.get_logger().warn(
                    f"No frame from {'tele' if tele else 'primary'} camera "
                    f"{device} using {profile_name}"
                )
                capture.release()
                continue

            frame_height, frame_width = first_frame.shape[:2]
            profile = f"{profile_name}:{frame_width}x{frame_height}"
            lock = self.tele_frame_lock if tele else self.frame_lock
            with lock:
                if tele:
                    self.latest_tele_frame = first_frame
                    self.tele_camera_profile = profile
                    self.tele_camera_resolution = [frame_width, frame_height]
                else:
                    self.latest_frame = first_frame
                    self.camera_profile = profile
                    self.camera_resolution = [frame_width, frame_height]
            self.get_logger().info(
                f"Opened {'tele' if tele else 'primary'} camera {device} as {profile}"
            )
            return capture

        if tele:
            self.tele_camera_profile = "unavailable"
            self.tele_camera_resolution = [0, 0]
        else:
            self.camera_profile = "unavailable"
            self.camera_resolution = [0, 0]
        return None

    def _capture_loop(self, tele: bool = False) -> None:
        while not self.stop_event.is_set():
            tele_required = self._tele_capture_required()
            capture_required = tele_required if tele else not tele_required
            if not capture_required:
                capture = self.tele_capture if tele else self.capture
                if capture is not None:
                    capture.release()
                    if tele:
                        self.tele_capture = None
                    else:
                        self.capture = None
                    self.get_logger().info(
                        f"Released {'tele' if tele else 'primary'} camera for USB handoff"
                    )
                if tele:
                    self.tele_camera_online = False
                    self.tele_camera_profile = "standby"
                    with self.tele_frame_lock:
                        self.latest_tele_frame = None
                else:
                    self.camera_online = False
                    self.camera_profile = "standby"
                    with self.frame_lock:
                        self.latest_frame = None
                time.sleep(0.2)
                continue
            other_capture = self.capture if tele else self.tele_capture
            if other_capture is not None:
                time.sleep(0.05)
                continue
            capture = self.tele_capture if tele else self.capture
            if capture is None:
                capture = self._open_camera(tele)
                if tele:
                    self.tele_capture = capture
                    self.tele_camera_online = capture is not None
                else:
                    self.capture = capture
                    self.camera_online = capture is not None
                if capture is None:
                    time.sleep(0.5)
                    continue
            ok, frame = capture.read()
            if not ok:
                if tele:
                    self.tele_camera_online = False
                    self.tele_capture.release()
                    self.tele_capture = None
                else:
                    self.camera_online = False
                    self.capture.release()
                    self.capture = None
                time.sleep(0.2)
                continue
            lock = self.tele_frame_lock if tele else self.frame_lock
            with lock:
                if tele:
                    self.latest_tele_frame = frame
                else:
                    self.latest_frame = frame

    def _tele_capture_required(self) -> bool:
        if not tele_camera_should_capture(self.task_state):
            return False
        if self.tele_scan_succeeded or self.tele_scan_timed_out:
            return False
        if self.tele_scan_started_s <= 0.0:
            self.tele_scan_started_s = time.monotonic()
        elapsed_s = time.monotonic() - self.tele_scan_started_s
        if elapsed_s < self.tele_scan_timeout_s:
            return True
        self.tele_scan_timed_out = True
        self.get_logger().warn(
            f"Tele QR scan timed out after {self.tele_scan_timeout_s:.1f}s; "
            "releasing tele camera and restoring primary camera"
        )
        return False

    def _tele_scan_state(self) -> str:
        if self.tele_scan_succeeded:
            return "success"
        if self.tele_scan_timed_out:
            return "failed"
        if tele_camera_should_capture(self.task_state):
            return "scanning"
        return "standby"

    def _preview_tick(self) -> None:
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        with self.tele_frame_lock:
            tele_frame = (
                None if self.latest_tele_frame is None else self.latest_tele_frame.copy()
            )

        if frame is not None:
            preview = self._resize_preview(frame)
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
            self._publish_preview(preview, self.preview_pub)

        if tele_frame is not None:
            tele_preview = self._resize_preview(tele_frame)
            self._publish_preview(tele_preview, self.tele_preview_pub)

    def _resize_preview(self, frame):
        height, width = frame.shape[:2]
        scale = min(
            self.preview_width / max(1, width),
            self.preview_height / max(1, height),
        )
        target_size = (
            max(1, round(width * scale)),
            max(1, round(height * scale)),
        )
        return cv2.resize(frame, target_size, interpolation=cv2.INTER_AREA)

    def _publish_preview(self, preview, publisher) -> None:
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
        publisher.publish(message)

    def _task_state(self, message: UInt8) -> None:
        state = int(message.data)
        if state != self.task_state:
            if state in (TASK_NAV_NURSE, TASK_WAIT_START):
                self.scan_values = dict.fromkeys(self.scan_values, "")
            if state == TASK_WAIT_START:
                self.tele_scan_started_s = time.monotonic()
                self.tele_scan_timed_out = False
                self.tele_scan_succeeded = False
            self.task_state = state
            self.consensus.reset()
            self.spatial_gate_open = False
            self.last_value = ""
            self.last_source = ""

    def _bridge_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            mission_epoch = int(payload["mission_epoch"])
            task_state = int((payload.get("stm32") or {}).get("task_state", -1))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        previous_epoch = self.mission_epoch
        self.mission_epoch = mission_epoch
        if previous_epoch is None or mission_epoch == previous_epoch:
            return
        if task_state != TASK_WAIT_START:
            return
        self.scan_values = dict.fromkeys(self.scan_values, "")
        self.tele_scan_started_s = time.monotonic()
        self.tele_scan_timed_out = False
        self.tele_scan_succeeded = False
        self.consensus.reset()
        self.spatial_gate_open = False
        self.last_value = ""
        self.last_source = ""
        self.get_logger().info(
            f"STM32 mission epoch changed {previous_epoch}->{mission_epoch}; "
            "restarting 15-second tele scan window"
        )
        self._publish_status()

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

    @staticmethod
    def _decode_barcode_fast(gray) -> Tuple[str, str]:
        for result in decode(gray, symbols=[ZBarSymbol.CODE128]):
            value = result.data.decode("ascii", "ignore").strip()
            if value_is_allowed(SCAN_FORMAT_CODE128, value):
                return value, "zbar_fast"
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
                return value, "zxing_code128_fast"
        return "", ""

    def _decode(self, gray, format: int) -> Tuple[str, str]:
        return self._decode_qr(gray) if format == SCAN_FORMAT_QR else self._decode_barcode(gray)

    def _decode_fast(self, gray, format: int) -> Tuple[str, str]:
        return (
            self._decode_qr(gray)
            if format == SCAN_FORMAT_QR
            else self._decode_barcode_fast(gray)
        )

    def _decode_tick(self) -> None:
        expected = expected_scan(self.task_state)
        if expected is None:
            return
        context, format = expected
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        with self.tele_frame_lock:
            tele_frame = (
                None if self.latest_tele_frame is None else self.latest_tele_frame.copy()
            )
        use_tele = tele_camera_assists(self.task_state, format)
        if frame is None and (not use_tele or tele_frame is None):
            return

        spatial_gate_open = self._scan_position_is_allowed(context)
        if spatial_gate_open != self.spatial_gate_open:
            self.consensus.reset()
            self.spatial_gate_open = spatial_gate_open
        if not spatial_gate_open:
            return

        started = time.perf_counter()
        value, source = "", ""
        x1 = y1 = x2 = y2 = 0
        if frame is not None:
            height, width = frame.shape[:2]
            left, right, top, bottom = self.roi
            x1, x2 = int(width * left), int(width * right)
            y1, y2 = int(height * top), int(height * bottom)
            full_frame = x1 == 0 and x2 == width and y1 == 0 and y2 == height
            scan_frame = frame if full_frame else frame[y1:y2, x1:x2]
            if (
                scan_frame.shape[1] > self.decode_width
                or scan_frame.shape[0] > self.decode_height
            ):
                fast_frame = cv2.resize(
                    scan_frame,
                    (self.decode_width, self.decode_height),
                    interpolation=cv2.INTER_AREA,
                )
            else:
                fast_frame = scan_frame
            fast_gray = cv2.cvtColor(fast_frame, cv2.COLOR_BGR2GRAY)
            value, source = self._decode_fast(fast_gray, format)
            if (
                not value
                and self.frame_index % self.full_resolution_fallback_period == 0
            ):
                full_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                value, source = self._decode(full_gray, format)
                if value:
                    source += "_fullres"
            elif (
                not value
                and not full_frame
                and self.frame_index % self.full_frame_period == 0
            ):
                full_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                value, source = self._decode_fast(full_gray, format)
                if value:
                    source += "_full"

        if not value and use_tele and tele_frame is not None:
            if (
                tele_frame.shape[1] > self.decode_width
                or tele_frame.shape[0] > self.decode_height
            ):
                tele_fast = cv2.resize(
                    tele_frame,
                    (self.decode_width, self.decode_height),
                    interpolation=cv2.INTER_AREA,
                )
            else:
                tele_fast = tele_frame
            tele_gray = cv2.cvtColor(tele_fast, cv2.COLOR_BGR2GRAY)
            value, source = self._decode_qr(tele_gray)
            if value:
                source = f"tele_{source}"
            elif self.frame_index % self.full_resolution_fallback_period == 0:
                tele_full_gray = cv2.cvtColor(tele_frame, cv2.COLOR_BGR2GRAY)
                value, source = self._decode_qr(tele_full_gray)
                if value:
                    source = f"tele_{source}_fullres"
        self.last_decode_ms = (time.perf_counter() - started) * 1000.0
        self.frame_index += 1

        if value and self.consensus.observe(format, value, time.monotonic()):
            self.last_value = value
            self.last_source = source
            self.scan_values[context] = value
            if (
                context == SCAN_CONTEXT_ORDER
                and self.task_state == TASK_WAIT_START
                and source.startswith("tele_")
            ):
                self.tele_scan_succeeded = True
            message = String()
            message.data = json.dumps(
                {"context": context, "format": format, "value": value, "source": source}
            )
            self.result_pub.publish(message)
            self.get_logger().info(
                f"Confirmed scan context={context} format={format} value={value} via {source}"
            )

        if self.preview and frame is not None:
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
                "camera_profile": self.camera_profile,
                "camera_resolution": self.camera_resolution,
                "tele_camera": self.tele_camera_online,
                "tele_device": self.tele_camera_device,
                "tele_camera_profile": self.tele_camera_profile,
                "tele_camera_resolution": self.tele_camera_resolution,
                "tele_scan_state": self._tele_scan_state(),
                "tele_scan_timeout_s": self.tele_scan_timeout_s,
                "tele_scan_elapsed_s": round(
                    max(0.0, time.monotonic() - self.tele_scan_started_s), 1
                )
                if self.tele_scan_started_s > 0.0
                else 0.0,
                "mission_epoch": self.mission_epoch,
                "tele_assist_active": bool(
                    expected and tele_camera_assists(self.task_state, expected[1])
                ),
                "task_state": self.task_state,
                "active": expected is not None,
                "context": context,
                "format": expected[1] if expected else 0,
                "value": self.last_value,
                "source": self.last_source,
                "decode_ms": round(self.last_decode_ms, 1),
                "decode_size": [self.decode_width, self.decode_height],
                "full_resolution_fallback_period": self.full_resolution_fallback_period,
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
        self.tele_capture_thread.join(timeout=2.0)
        if self.capture is not None:
            self.capture.release()
        if self.tele_capture is not None:
            self.tele_capture.release()
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
