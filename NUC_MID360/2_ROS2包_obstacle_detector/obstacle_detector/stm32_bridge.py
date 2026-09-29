#!/usr/bin/env python3
"""STM32 <-> Nav2 bridge.

Outbound (STM32 -> Nav2)
    MSG_POSE supplies OPS9 position and HWT101CT yaw. MSG_WHEEL_ODOM supplies
    encoder-derived body velocity. A planar EKF fuses both into odom->base_link
    and Odometry, with pose differentiation retained as a firmware fallback.

Inbound (Nav2 -> STM32)
    /cmd_vel_safe from Collision Monitor becomes MSG_VEL_CMD frames. A watchdog sends an explicit zero
    command when Nav2 stops publishing, so a dead planner cannot leave the
    chassis driving.

Frame conventions. The STM32 reports x right / y forward / yaw clockwise.
    ROS wants x forward / y left / yaw counter-clockwise. The Mid360 is handled
    exclusively by robot_state_publisher TF; this bridge never rewrites points.
"""

import json
import math
import os
import struct
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from typing import Deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String, UInt8
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .nav_protocol import (
    GOAL_NONE,
    Frame,
    MSG_GOAL_REQUEST,
    MSG_HEARTBEAT,
    MSG_NAV_STATUS,
    MSG_POSE,
    MSG_SCAN_ACK,
    MSG_SCAN_RESULT,
    MSG_STP23L,
    MSG_TTS_REQUEST,
    MSG_TTS_STATUS,
    MSG_VEL_CMD,
    MSG_WHEEL_DIAGNOSTICS,
    MSG_WHEEL_ODOM,
    NAV_FOLLOWING,
    NAV_IDLE,
    NAV_WAIT_PATH,
    SCAN_ACK_ACCEPTED,
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_BED3,
    SCAN_CONTEXT_ORDER,
    START_WAIT_TASK_STATES,
    TTS_STATUS_COMPLETED,
    TTS_STATUS_ERROR,
    decode_goal_request,
    decode_pose,
    decode_scan_ack,
    decode_stp23l,
    decode_tts_request,
    decode_wheel_diagnostics,
    decode_wheel_odom,
    encode_frame,
    encode_heartbeat,
    encode_nav_status,
    encode_scan_result,
    encode_tts_status,
    encode_velocity,
)
from .bridge_safety import (
    GateReleaseLimiter,
    SettledStopDetector,
    medical_mission_restarted,
    navigation_motion_is_authorized,
)
from .field_goals import load_field_goals
from .nuc_tts import NucTtsPlayer, TtsPlaybackResult
from .scanner_core import (
    TASK_NAV_NURSE,
    TASK_WAIT_START,
    scan_matches_task,
    scan_position_is_allowed,
)
from .serial_transport import SerialTransport
from .stp23l_calibration import (
    CalibrationConfig,
    OpsRangeCalibrator,
    load_calibration_config,
)
from .wheel_odometry_ekf import (
    PlanarWheelOdometryEkf,
    WheelOdometryEkfConfig,
)


POINT_NAMES = {
    GOAL_NONE: "none",
    1: "home",
    2: "nurse",
    3: "bed1",
    4: "bed3",
}

class Stm32Bridge(Node):
    def __init__(self) -> None:
        super().__init__("stm32_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        # 1.0 = STM32 yaw already counter-clockwise, -1.0 = clockwise as shipped.
        self.declare_parameter("yaw_sign", -1.0)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("cmd_timeout_s", 0.4)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("heartbeat_rate_hz", 10.0)
        self.declare_parameter("scan_retry_s", 0.15)
        self.declare_parameter("bed_scan_activation_distance_m", 1.2)
        self.declare_parameter("goal_handoff_hold_s", 0.5)
        self.declare_parameter("navigator_following_hold_s", 0.3)
        self.declare_parameter("navigator_status_timeout_s", 1.2)
        self.declare_parameter("gate_release_wheel_accel_m_s2", 2.5)
        self.declare_parameter("gate_release_yaw_radius_m", 0.25)
        self.declare_parameter("gate_release_rearm_drop_m_s", 0.25)
        self.declare_parameter("nurse_scan_stop_linear_m_s", 0.03)
        self.declare_parameter("nurse_scan_stop_angular_rad_s", 0.05)
        self.declare_parameter("nurse_scan_stop_settle_s", 0.15)
        self.declare_parameter("twist_filter_alpha", 0.35)
        self.declare_parameter("wheel_odom_timeout_s", 0.15)
        self.declare_parameter("ekf_ops_position_std_m", 0.02)
        self.declare_parameter("ekf_hwt_yaw_std_rad", 0.015)
        self.declare_parameter("ekf_wheel_forward_std_m_s", 0.08)
        self.declare_parameter("ekf_wheel_lateral_std_m_s", 0.16)
        self.declare_parameter("ekf_wheel_yaw_std_rad_s", 0.12)
        self.declare_parameter("ekf_linear_accel_std_m_s2", 1.5)
        self.declare_parameter("ekf_yaw_accel_std_rad_s2", 1.5)
        self.declare_parameter(
            "tts_bed1_audio_file", "/home/fzurobot/Downloads/1_.mp3"
        )
        self.declare_parameter(
            "tts_bed3_audio_file", "/home/fzurobot/Downloads/3_.mp3"
        )
        self.declare_parameter("tts_volume_percent", 100)
        self.declare_parameter("tts_audio_device", "pulse")
        self.declare_parameter("tts_lead_silence_s", 0.0)
        self.declare_parameter("tts_tail_silence_s", 0.0)
        self.declare_parameter("tts_keepalive_enabled", True)
        self.declare_parameter("tts_timeout_s", 8.0)
        # NUC-side linear clamp: 2.00 m/s per body-axis component. The
        # STM32 applies its own independent 4.00 m/s hard cap.
        self.declare_parameter("max_speed_mm_s", 2000.0)
        self.declare_parameter("max_yaw_cdeg_s", 9000.0)
        # Competition mode also applies when this node is launched directly.
        self.declare_parameter("dry_run", False)
        self.declare_parameter("enforce_task_gate", True)
        default_field_config = os.path.join(
            get_package_share_directory("obstacle_detector"),
            "config",
            "field_map.yaml",
        )
        self.declare_parameter("field_config", default_field_config)

        get = lambda name: self.get_parameter(name).value
        self.map_frame = str(get("map_frame"))
        self.odom_frame = str(get("odom_frame"))
        self.base_frame = str(get("base_frame"))
        self.cmd_vel_topic = str(get("cmd_vel_topic"))
        self.yaw_sign = float(get("yaw_sign"))
        self.pose_timeout_s = float(get("pose_timeout_s"))
        self.cmd_timeout_s = float(get("cmd_timeout_s"))
        self.max_speed_mm_s = float(get("max_speed_mm_s"))
        self.max_yaw_cdeg_s = float(get("max_yaw_cdeg_s"))
        self.twist_filter_alpha = max(0.0, min(1.0, float(get("twist_filter_alpha"))))
        self.wheel_odom_timeout_s = max(
            0.05, float(get("wheel_odom_timeout_s"))
        )
        self.scan_retry_s = max(0.05, float(get("scan_retry_s")))
        self.bed_scan_activation_distance_mm = max(
            0.1, float(get("bed_scan_activation_distance_m"))
        ) * 1000.0
        self.goal_handoff_hold_s = max(0.0, float(get("goal_handoff_hold_s")))
        self.navigator_following_hold_s = max(
            0.0, float(get("navigator_following_hold_s"))
        )
        self.navigator_status_timeout_s = max(
            0.5, float(get("navigator_status_timeout_s"))
        )
        self.gate_release_limiter = GateReleaseLimiter(
            float(get("gate_release_wheel_accel_m_s2")),
            float(get("gate_release_yaw_radius_m")),
            float(get("gate_release_rearm_drop_m_s")),
        )
        self.nurse_scan_stop_detector = SettledStopDetector(
            float(get("nurse_scan_stop_linear_m_s")),
            float(get("nurse_scan_stop_angular_rad_s")),
            float(get("nurse_scan_stop_settle_s")),
        )
        self.dry_run = bool(get("dry_run"))
        self.enforce_task_gate = bool(get("enforce_task_gate"))
        field_config = str(get("field_config"))
        goals_by_name, _ = load_field_goals(field_config)
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
        try:
            calibration_config = load_calibration_config(field_config)
        except (OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"STP23L calibration disabled: cannot load {field_config}: {exc}"
            )
            calibration_config = CalibrationConfig(
                False, 153.0, 7, 40.0, 200.0, 8.0, 400.0, ()
            )
        self.calibrator = OpsRangeCalibrator(calibration_config)
        self.sensor_radius_m = calibration_config.sensor_radius_mm / 1000.0
        self.odom_ekf = PlanarWheelOdometryEkf(
            WheelOdometryEkfConfig(
                ops_position_std_m=max(
                    0.001, float(get("ekf_ops_position_std_m"))
                ),
                hwt_yaw_std_rad=max(
                    0.001, float(get("ekf_hwt_yaw_std_rad"))
                ),
                wheel_forward_std_m_s=max(
                    0.001, float(get("ekf_wheel_forward_std_m_s"))
                ),
                wheel_lateral_std_m_s=max(
                    0.001, float(get("ekf_wheel_lateral_std_m_s"))
                ),
                wheel_yaw_std_rad_s=max(
                    0.001, float(get("ekf_wheel_yaw_std_rad_s"))
                ),
                linear_accel_std_m_s2=max(
                    0.01, float(get("ekf_linear_accel_std_m_s2"))
                ),
                yaw_accel_std_rad_s2=max(
                    0.01, float(get("ekf_yaw_accel_std_rad_s2"))
                ),
            )
        )

        self.pose = None
        self.previous_ros_pose = None
        self.twist = (0.0, 0.0, 0.0)
        self.last_pose_s = 0.0
        self.last_cmd_s = 0.0
        self.last_sent = (0.0, 0.0, 0.0)
        self.last_safe_cmd = (0.0, 0.0, 0.0)
        self.nurse_scan_soft_stop = False
        self.tx_seq = 0
        self.heartbeat_counter = 0
        self.stopped = False
        self.cmd_count = 0
        self.rx_queue: Deque[Frame] = deque()
        self.started_s = time.monotonic()
        self.stp23l = None
        self.wheel_odom = None
        self.wheel_diagnostics = None
        self.last_wheel_odom_s = 0.0
        self.last_calibration_event = None
        self.next_scan_id = 0
        self.pending_scan = None
        self.last_scan_ack = None
        self.requested_request_id = 0
        self.requested_goal_id = GOAL_NONE
        self.navigator_request_id = 0
        self.navigator_goal_id = GOAL_NONE
        self.navigator_state = NAV_IDLE
        self.navigator_plan_ready = False
        self.goal_request_s = 0.0
        self.last_navigator_status_s = 0.0
        self.navigator_following_s = 0.0
        self.mission_epoch = 0
        self.tts_player = NucTtsPlayer(
            bed1_audio_file=str(get("tts_bed1_audio_file")),
            bed3_audio_file=str(get("tts_bed3_audio_file")),
            volume_percent=int(get("tts_volume_percent")),
            audio_device=str(get("tts_audio_device")),
            lead_silence_s=float(get("tts_lead_silence_s")),
            tail_silence_s=float(get("tts_tail_silence_s")),
            keepalive_enabled=bool(get("tts_keepalive_enabled")),
            timeout_s=float(get("tts_timeout_s")),
        )
        self.tts_keepalive_timer = self.create_timer(
            5.0, self._maintain_tts_keepalive
        )
        self._maintain_tts_keepalive()
        self.tts_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="nuc-tts"
        )
        self.tts_future = None
        self.tts_active_key = None
        self.tts_results = {}
        self.last_tts_status = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/medical_nav/robot_pose", 10)
        self.raw_pose_pub = self.create_publisher(
            PoseStamped, "/medical_nav/ops_raw_pose", 10
        )
        self.wheel_odom_pub = self.create_publisher(
            Odometry, "/medical_nav/wheel_odom", 10
        )
        self.wheel_diagnostics_pub = self.create_publisher(
            String, "/medical_nav/wheel_diagnostics", 10
        )
        self.status_pub = self.create_publisher(String, "/medical_nav/bridge_status", 10)
        self.goal_pub = self.create_publisher(String, "/medical_nav/goal_request", 10)
        self.stp23l_pub = self.create_publisher(String, "/medical_nav/stp23l", 10)
        self.calibration_pub = self.create_publisher(
            String, "/medical_nav/ops_calibration", 10
        )
        self.task_state_pub = self.create_publisher(UInt8, "/medical_nav/task_state", 10)
        self.scan_transport_pub = self.create_publisher(
            String, "/medical_nav/scan_transport_status", 10
        )
        self.range_pubs = {
            "a": self.create_publisher(Range, "/medical_nav/stp23l/right", 10),
            "b": self.create_publisher(Range, "/medical_nav/stp23l/front", 10),
            "c": self.create_publisher(Range, "/medical_nav/stp23l/left", 10),
        }

        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel, 10)
        self.create_subscription(
            String, "/medical_nav/navigator_status", self._navigator_status, 10
        )
        self.create_subscription(
            String, "/medical_nav/scan_result", self._scanner_result, 10
        )

        self.transport = SerialTransport(
            str(get("serial_port")),
            int(get("baud_rate")),
            self._queue_frame,
            self.get_logger(),
        )
        self.transport.start()
        self._publish_static_map_to_odom()

        rate = max(5.0, float(get("publish_rate_hz")))
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0 / max(1.0, float(get("heartbeat_rate_hz"))), self._heartbeat)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"STM32 bridge ready | {self.map_frame}->{self.odom_frame}->{self.base_frame} "
            f"| velocity={self.cmd_vel_topic} | yaw_sign={self.yaw_sign:+.0f} "
            f"| dry_run={self.dry_run} | task_gate={self.enforce_task_gate}"
        )

    # ------------------------------------------------------------------
    # Serial receive
    # ------------------------------------------------------------------

    def _queue_frame(self, frame: Frame) -> None:
        self.rx_queue.append(frame)

    def _drain_frames(self) -> None:
        while self.rx_queue:
            frame = self.rx_queue.popleft()
            try:
                if frame.msg_type == MSG_POSE:
                    self._handle_pose(frame)
                elif frame.msg_type == MSG_GOAL_REQUEST:
                    self._handle_goal_request(frame)
                elif frame.msg_type == MSG_STP23L:
                    self._handle_stp23l(frame)
                elif frame.msg_type == MSG_WHEEL_ODOM:
                    self._handle_wheel_odom(frame)
                elif frame.msg_type == MSG_WHEEL_DIAGNOSTICS:
                    self._handle_wheel_diagnostics(frame)
                elif frame.msg_type == MSG_SCAN_ACK:
                    self._handle_scan_ack(frame)
                elif frame.msg_type == MSG_TTS_REQUEST:
                    self._handle_tts_request(frame)
            except (ValueError, struct.error) as exc:
                self.get_logger().warn(
                    f"Rejected frame 0x{frame.msg_type:02X}: {exc}", throttle_duration_sec=2.0
                )

    def _handle_pose(self, frame: Frame) -> None:
        previous_task_state = self.pose.task_state if self.pose is not None else None
        previous_nav_status = self.pose.nav_status if self.pose is not None else None
        pose = decode_pose(frame.payload)
        received_s = time.monotonic()
        pose_gap_s = (
            received_s - self.last_pose_s
            if self.last_pose_s > 0.0
            else float("inf")
        )
        mission_restarted = medical_mission_restarted(
            previous_task_state,
            previous_nav_status,
            pose.task_state,
            pose.nav_status,
            pose_gap_s,
            self.pose_timeout_s,
        )
        if mission_restarted:
            self.gate_release_limiter.reset(received_s)
            self.mission_epoch = (self.mission_epoch + 1) & 0xFFFFFFFF
            if self.mission_epoch == 0:
                self.mission_epoch = 1
            self.requested_request_id = 0
            self.requested_goal_id = GOAL_NONE
            self.navigator_request_id = 0
            self.navigator_goal_id = GOAL_NONE
            self.navigator_state = NAV_IDLE
            self.navigator_plan_ready = False
            self.goal_request_s = 0.0
            self.last_navigator_status_s = 0.0
            self.navigator_following_s = 0.0
            self.pending_scan = None
            self.last_scan_ack = None
            self.tts_results.clear()
            self.last_tts_status = None
            self.nurse_scan_soft_stop = False
            self.nurse_scan_stop_detector.reset()
            self.calibrator.reset()
            self.odom_ekf.reset()
            self.previous_ros_pose = None
            self.twist = (0.0, 0.0, 0.0)
            self.get_logger().info(
                f"New medical mission epoch={self.mission_epoch}: "
                "cleared stale navigation, scan and calibration state"
            )
        elif (
            pose.task_state in (TASK_WAIT_START, TASK_NAV_NURSE)
            and previous_task_state not in (TASK_WAIT_START, TASK_NAV_NURSE)
        ):
            self.calibrator.reset()
            self.odom_ekf.reset()
            self.previous_ros_pose = None
            self.twist = (0.0, 0.0, 0.0)
        x, y, yaw = self._as_ros(pose.x_mm, pose.y_mm, pose.yaw_cdeg)

        if self.previous_ros_pose is not None:
            old_x, old_y, old_yaw, old_s = self.previous_ros_pose
            dt = received_s - old_s
            if 0.01 <= dt <= self.pose_timeout_s:
                world_vx = (x - old_x) / dt
                world_vy = (y - old_y) / dt
                dyaw = math.atan2(math.sin(yaw - old_yaw), math.cos(yaw - old_yaw))
                raw_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
                raw_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
                raw_wz = dyaw / dt
                # Reject coordinate resets and corrupt frames instead of feeding a
                # one-cycle velocity spike into MPPI.
                if math.hypot(raw_vx, raw_vy) <= 2.0 and abs(raw_wz) <= 6.0:
                    alpha = self.twist_filter_alpha
                    self.twist = tuple(
                        alpha * new + (1.0 - alpha) * old
                        for new, old in zip((raw_vx, raw_vy, raw_wz), self.twist)
                    )
                else:
                    self.twist = (0.0, 0.0, 0.0)
            elif dt > self.pose_timeout_s:
                self.twist = (0.0, 0.0, 0.0)

        self.previous_ros_pose = (x, y, yaw, received_s)
        self.odom_ekf.update_pose(x, y, yaw)
        if self._wheel_odom_is_fresh():
            self.twist = self.odom_ekf.twist
        self.pose = pose
        self.last_pose_s = received_s
        if previous_task_state is not None and previous_task_state != pose.task_state:
            self.stopped = True
            self.gate_release_limiter.reset(received_s)
            self._send_velocity(0.0, 0.0, 0.0, source="task_transition")
        task_message = UInt8()
        task_message.data = pose.task_state
        self.task_state_pub.publish(task_message)
        if previous_task_state != pose.task_state and self.pending_scan is not None:
            pending = self.pending_scan
            if not scan_matches_task(
                pose.task_state,
                pending["context"],
                pending["format"],
                pending["value"],
            ):
                self.pending_scan = None
                self.nurse_scan_soft_stop = False
                self.nurse_scan_stop_detector.reset()

    def _handle_goal_request(self, frame: Frame) -> None:
        request = decode_goal_request(frame.payload)
        if request.goal_id == GOAL_NONE:
            return
        request_changed = (
            request.request_id != self.requested_request_id
            or request.goal_id != self.requested_goal_id
        )
        if request_changed:
            self.requested_request_id = request.request_id
            self.requested_goal_id = request.goal_id
            self.navigator_state = NAV_WAIT_PATH
            self.goal_request_s = time.monotonic()
            self.navigator_request_id = 0
            self.navigator_goal_id = GOAL_NONE
            self.navigator_plan_ready = False
            self.last_navigator_status_s = 0.0
            self.navigator_following_s = 0.0
            self.stopped = True
            self.gate_release_limiter.reset(self.goal_request_s)
            self._send_velocity(0.0, 0.0, 0.0, source="goal_handoff")
        name = POINT_NAMES.get(request.goal_id, str(request.goal_id))
        message = String()
        message.data = json.dumps(
            {"request_id": request.request_id, "goal_id": request.goal_id, "point": name}
        )
        self.goal_pub.publish(message)
        self.get_logger().info(
            f"STM32 goal request #{request.request_id} -> {name}", throttle_duration_sec=1.0
        )

    def _handle_wheel_odom(self, frame: Frame) -> None:
        telemetry = decode_wheel_odom(frame.payload)
        received_s = time.monotonic()
        forward_m_s = telemetry.forward_mm_s / 1000.0
        left_m_s = telemetry.left_mm_s / 1000.0
        yaw_ccw_rad_s = math.radians(telemetry.yaw_ccw_cdeg_s / 100.0)
        accepted = self.odom_ekf.update_wheel(
            forward_m_s,
            left_m_s,
            yaw_ccw_rad_s,
            telemetry.online_mask,
            telemetry.stamp_cs,
        )
        self.wheel_odom = telemetry
        self.last_wheel_odom_s = received_s
        if accepted and self.odom_ekf.initialized:
            self.twist = self.odom_ekf.twist

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.twist.twist.linear.x = forward_m_s
        message.twist.twist.linear.y = left_m_s
        message.twist.twist.angular.z = yaw_ccw_rad_s
        valid = telemetry.online_mask == 0x0F
        invalid_variance = 1.0e6
        message.pose.covariance[0] = invalid_variance
        message.pose.covariance[7] = invalid_variance
        message.pose.covariance[35] = invalid_variance
        message.twist.covariance[0] = (
            self.odom_ekf.config.wheel_forward_std_m_s**2
            if valid
            else invalid_variance
        )
        message.twist.covariance[7] = (
            self.odom_ekf.config.wheel_lateral_std_m_s**2
            if valid
            else invalid_variance
        )
        message.twist.covariance[35] = (
            self.odom_ekf.config.wheel_yaw_std_rad_s**2
            if valid
            else invalid_variance
        )
        self.wheel_odom_pub.publish(message)

    def _handle_wheel_diagnostics(self, frame: Frame) -> None:
        telemetry = decode_wheel_diagnostics(frame.payload)
        self.wheel_diagnostics = telemetry
        message = String()
        message.data = json.dumps(
            {
                "target_rpm": telemetry.target_rpm,
                "measured_rpm": telemetry.measured_rpm,
                "rpm_error": [
                    target - measured
                    for target, measured in zip(
                        telemetry.target_rpm, telemetry.measured_rpm
                    )
                ],
                "command_current": telemetry.command_current,
                "feedback_current": telemetry.feedback_current,
                "online_mask": telemetry.online_mask,
                "current_saturation_mask": telemetry.current_saturation_mask,
                "stamp_cs": telemetry.stamp_cs,
            },
            separators=(",", ":"),
        )
        self.wheel_diagnostics_pub.publish(message)

    def _handle_stp23l(self, frame: Frame) -> None:
        telemetry = decode_stp23l(frame.payload)
        self.stp23l = telemetry
        stamp = self.get_clock().now().to_msg()
        sensor_data = {
            "a": ("right", "stp23l_right", telemetry.a_mm, 0x01),
            "b": ("front", "stp23l_front", telemetry.b_mm, 0x02),
            "c": ("left", "stp23l_left", telemetry.c_mm, 0x04),
        }
        aggregate = {"valid_mask": telemetry.valid_mask, "sensors": {}}
        for sensor, (direction, frame_id, distance_mm, bit) in sensor_data.items():
            valid = bool(telemetry.valid_mask & bit) and distance_mm > 0
            message = Range()
            message.header.stamp = stamp
            message.header.frame_id = frame_id
            message.radiation_type = Range.INFRARED
            message.field_of_view = 0.0
            message.min_range = 0.1
            message.max_range = 13.4
            message.range = distance_mm / 1000.0 if valid else float("nan")
            self.range_pubs[sensor].publish(message)
            aggregate["sensors"][sensor] = {
                "direction": direction,
                "valid": valid,
                "sensor_distance_mm": distance_mm if valid else None,
                "center_distance_mm": (
                    round(distance_mm + self.sensor_radius_m * 1000.0, 1)
                    if valid
                    else None
                ),
            }
        aggregate_message = String()
        aggregate_message.data = json.dumps(aggregate, ensure_ascii=False)
        self.stp23l_pub.publish(aggregate_message)

        if self.pose is None:
            return
        event = self.calibrator.update(
            self.pose.task_state,
            self.pose.nav_status,
            self.pose.x_mm,
            self.pose.y_mm,
            self.pose.yaw_cdeg,
            telemetry,
        )
        self.last_calibration_event = event
        calibration = asdict(event)
        calibration["offset_x_mm"] = round(self.calibrator.offset_x_mm, 1)
        calibration["offset_y_mm"] = round(self.calibrator.offset_y_mm, 1)
        calibration_message = String()
        calibration_message.data = json.dumps(calibration, ensure_ascii=False)
        self.calibration_pub.publish(calibration_message)
        if event.applied:
            # Do not differentiate the deliberate pose correction into a
            # one-cycle velocity spike.
            self.previous_ros_pose = None
            self.twist = (0.0, 0.0, 0.0)
            self.odom_ekf.reset()
            corrected_x, corrected_y, corrected_yaw = self._as_ros(
                self.pose.x_mm, self.pose.y_mm, self.pose.yaw_cdeg
            )
            self.odom_ekf.update_pose(
                corrected_x, corrected_y, corrected_yaw
            )
            self.get_logger().info(
                f"STP23L calibrated at {event.bed}: "
                f"OPS offset=({event.offset_x_mm:.1f}, {event.offset_y_mm:.1f}) mm, "
                f"field pose=({event.measured_x_mm:.1f}, {event.measured_y_mm:.1f}) mm"
            )

    def _handle_scan_ack(self, frame: Frame) -> None:
        ack = decode_scan_ack(frame.payload)
        if self.pending_scan is None or ack.scan_id != self.pending_scan["scan_id"]:
            return
        self.last_scan_ack = {"scan_id": ack.scan_id, "status": ack.status}
        message = String()
        message.data = json.dumps(self.last_scan_ack)
        self.scan_transport_pub.publish(message)
        if ack.status == SCAN_ACK_ACCEPTED:
            self.get_logger().info(
                f"STM32 accepted scan #{ack.scan_id}: {self.pending_scan['value']}"
            )
        else:
            self.get_logger().warn(
                f"STM32 rejected scan #{ack.scan_id} with status {ack.status}"
            )
        self.pending_scan = None
        self.nurse_scan_soft_stop = False
        self.nurse_scan_stop_detector.reset()

    def _handle_tts_request(self, frame: Frame) -> None:
        request = decode_tts_request(frame.payload)
        key = (self.mission_epoch, request.request_id, request.bed)

        self._poll_tts()
        cached_status = self.tts_results.get(key)
        if cached_status is not None:
            self._send_tts_status(request.request_id, cached_status)
            return

        if self.tts_future is not None:
            if self.tts_active_key != key:
                self.get_logger().warn(
                    "Deferred TTS request while another announcement is playing",
                    throttle_duration_sec=1.0,
                )
            return

        self.tts_active_key = key
        self.tts_future = self.tts_executor.submit(
            self.tts_player.speak_bed, request.bed
        )
        self.get_logger().info(
            f"Started NUC TTS request={request.request_id} bed={request.bed}"
        )

    def _maintain_tts_keepalive(self) -> None:
        was_active = self.tts_player.keepalive_active
        result = self.tts_player.start_keepalive()
        is_active = self.tts_player.keepalive_active
        if is_active and not was_active:
            self.get_logger().info(result.detail)
        elif not result.success:
            self.get_logger().warn(
                f"NUC audio keepalive failed: {result.detail}",
                throttle_duration_sec=5.0,
            )

    def _poll_tts(self) -> None:
        if self.tts_future is None or not self.tts_future.done():
            return

        key = self.tts_active_key
        try:
            result = self.tts_future.result()
        except Exception as exc:
            result = TtsPlaybackResult(False, str(exc))
        self.tts_future = None
        self.tts_active_key = None
        if key is None or key[0] != self.mission_epoch:
            return

        _, request_id, bed = key
        status = TTS_STATUS_COMPLETED if result.success else TTS_STATUS_ERROR
        self.tts_results[key] = status
        while len(self.tts_results) > 8:
            self.tts_results.pop(next(iter(self.tts_results)))
        self.last_tts_status = {
            "request_id": request_id,
            "bed": bed,
            "status": status,
            "detail": result.detail,
        }
        self._send_tts_status(request_id, status)
        if result.success:
            self.get_logger().info(
                f"Completed NUC TTS request={request_id} bed={bed}: "
                f"{result.detail}"
            )
        else:
            self.get_logger().error(
                f"NUC TTS failed request={request_id} bed={bed}: {result.detail}"
            )

    def _send_tts_status(self, request_id: int, status: int) -> None:
        payload = encode_tts_status(request_id, status)
        frame = encode_frame(MSG_TTS_STATUS, self.tx_seq, payload)
        if self.transport.send(frame):
            self.tx_seq = (self.tx_seq + 1) & 0xFF

    def _scanner_result(self, message: String) -> None:
        try:
            result = json.loads(message.data)
            context = int(result["context"])
            scan_format = int(result["format"])
            value = str(result["value"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Bad scanner result: {exc}")
            return
        if not self._pose_is_fresh() or not scan_matches_task(
            self.pose.task_state, context, scan_format, value
        ):
            self.get_logger().warn(
                f"Ignored scan outside expected task state: state="
                f"{self.pose.task_state if self.pose else 'none'} value={value}"
            )
            return
        corrected_x_mm, corrected_y_mm = self.calibrator.corrected_xy(
            self.pose.x_mm, self.pose.y_mm
        )
        if not scan_position_is_allowed(
            context,
            corrected_x_mm,
            corrected_y_mm,
            self.scan_targets_mm,
            self.bed_scan_activation_distance_mm,
        ):
            self.get_logger().warn(
                f"Ignored bed scan outside target area: context={context} "
                f"pose=({corrected_x_mm:.0f}, {corrected_y_mm:.0f}) value={value}"
            )
            return
        if self.pending_scan is not None:
            return

        self.next_scan_id = (self.next_scan_id + 1) & 0xFFFF
        if self.next_scan_id == 0:
            self.next_scan_id = 1
        payload = encode_scan_result(
            self.next_scan_id, context, scan_format, value
        )
        self.pending_scan = {
            "scan_id": self.next_scan_id,
            "context": context,
            "format": scan_format,
            "value": value,
            "payload": payload,
            "last_sent_s": 0.0,
            "attempts": 0,
            "defer_until_stopped": context == SCAN_CONTEXT_ORDER
            and self.pose.task_state == TASK_NAV_NURSE,
        }
        if self.pending_scan["defer_until_stopped"]:
            self.nurse_scan_soft_stop = True
            self.nurse_scan_stop_detector.reset()
            self.get_logger().info(
                "Nurse QR latched; waiting for /cmd_vel_safe to settle before STM32 ACK"
            )
        else:
            self._send_pending_scan(force=True)

    def _send_pending_scan(self, force=False) -> None:
        if self.pending_scan is None:
            return
        now = time.monotonic()
        deferred = bool(self.pending_scan.get("defer_until_stopped", False))
        if deferred and not self.nurse_scan_stop_detector.update(
            *self.last_safe_cmd, now
        ):
            return
        if not force and now - self.pending_scan["last_sent_s"] < self.scan_retry_s:
            return
        frame = encode_frame(MSG_SCAN_RESULT, self.tx_seq, self.pending_scan["payload"])
        if self.transport.send(frame):
            self.tx_seq = (self.tx_seq + 1) & 0xFF
            self.pending_scan["last_sent_s"] = now
            self.pending_scan["attempts"] += 1
            if deferred:
                self.pending_scan["defer_until_stopped"] = False
                self.nurse_scan_soft_stop = False
                self.get_logger().info(
                    "Nurse soft stop settled; forwarding QR result to STM32"
                )

    # ------------------------------------------------------------------
    # Pose out
    # ------------------------------------------------------------------

    def _pose_is_fresh(self) -> bool:
        return (
            self.pose is not None
            and time.monotonic() - self.last_pose_s <= self.pose_timeout_s
        )

    def _wheel_odom_is_fresh(self) -> bool:
        return (
            self.wheel_odom is not None
            and self.wheel_odom.online_mask == 0x0F
            and self.odom_ekf.wheel_valid
            and time.monotonic() - self.last_wheel_odom_s
            <= self.wheel_odom_timeout_s
        )

    def _as_ros(self, x_mm: float, y_mm: float, yaw_cdeg: float, corrected=True):
        """STM32 field pose -> ROS map pose (metres, counter-clockwise yaw)."""
        if corrected:
            x_mm, y_mm = self.calibrator.corrected_xy(x_mm, y_mm)
        return (
            y_mm / 1000.0,
            -x_mm / 1000.0,
            self.yaw_sign * math.radians(yaw_cdeg / 100.0),
        )

    def _publish_static_map_to_odom(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        transforms = [transform]
        for frame_id, x, y, yaw in (
            ("stp23l_front", self.sensor_radius_m, 0.0, 0.0),
            ("stp23l_right", 0.0, -self.sensor_radius_m, -math.pi / 2.0),
            ("stp23l_left", 0.0, self.sensor_radius_m, math.pi / 2.0),
        ):
            sensor_tf = TransformStamped()
            sensor_tf.header.stamp = transform.header.stamp
            sensor_tf.header.frame_id = self.base_frame
            sensor_tf.child_frame_id = frame_id
            sensor_tf.transform.translation.x = x
            sensor_tf.transform.translation.y = y
            sensor_tf.transform.rotation.z = math.sin(yaw * 0.5)
            sensor_tf.transform.rotation.w = math.cos(yaw * 0.5)
            transforms.append(sensor_tf)
        self.static_tf_broadcaster.sendTransform(transforms)

    def _tick(self) -> None:
        self._drain_frames()
        self._poll_tts()
        self._send_pending_scan()
        if self._pose_is_fresh():
            self._publish_pose()
        self._command_watchdog()

    def _publish_pose(self) -> None:
        fused = self._wheel_odom_is_fresh() and self.odom_ekf.initialized
        if fused:
            x, y, yaw = self.odom_ekf.pose
            self.twist = self.odom_ekf.twist
        else:
            x, y, yaw = self._as_ros(
                self.pose.x_mm, self.pose.y_mm, self.pose.yaw_cdeg
            )
        stamp = self.get_clock().now().to_msg()
        half = yaw * 0.5
        qz, qw = math.sin(half), math.cos(half)

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame
        odom_to_base.transform.translation.x = x
        odom_to_base.transform.translation.y = y
        odom_to_base.transform.rotation.z = qz
        odom_to_base.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(odom_to_base)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        odometry.twist.twist.linear.x = self.twist[0]
        odometry.twist.twist.linear.y = self.twist[1]
        odometry.twist.twist.angular.z = self.twist[2]
        if fused:
            covariance = self.odom_ekf.covariance
            odometry.pose.covariance[0] = float(covariance[0, 0])
            odometry.pose.covariance[7] = float(covariance[1, 1])
            odometry.pose.covariance[35] = float(covariance[2, 2])
            odometry.twist.covariance[0] = float(covariance[3, 3])
            odometry.twist.covariance[7] = float(covariance[4, 4])
            odometry.twist.covariance[35] = float(covariance[5, 5])
        self.odom_pub.publish(odometry)

        pose_message = PoseStamped()
        pose_message.header = odometry.header
        pose_message.pose = odometry.pose.pose
        self.pose_pub.publish(pose_message)

        raw_x, raw_y, raw_yaw = self._as_ros(
            self.pose.x_mm, self.pose.y_mm, self.pose.yaw_cdeg, corrected=False
        )
        raw_pose = PoseStamped()
        raw_pose.header = odometry.header
        raw_pose.pose.position.x = raw_x
        raw_pose.pose.position.y = raw_y
        raw_pose.pose.orientation.z = math.sin(raw_yaw * 0.5)
        raw_pose.pose.orientation.w = math.cos(raw_yaw * 0.5)
        self.raw_pose_pub.publish(raw_pose)

    # ------------------------------------------------------------------
    # Velocity in
    # ------------------------------------------------------------------

    def _cmd_vel(self, msg: Twist) -> None:
        now_s = time.monotonic()
        self.last_cmd_s = now_s
        self.last_safe_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        if (
            self.nurse_scan_soft_stop
            and self._pose_is_fresh()
            and self.pose.task_state == TASK_NAV_NURSE
        ):
            self.stopped = False
            self.gate_release_limiter.reset(now_s)
            self._send_velocity(
                msg.linear.x,
                msg.linear.y,
                msg.angular.z,
                source="nurse_scan_soft_stop",
            )
            return
        if not self._motion_is_authorized():
            self.stopped = True
            self.gate_release_limiter.reset(now_s)
            self._send_velocity(0.0, 0.0, 0.0, source="task_gate")
            return
        self.stopped = False
        if self.enforce_task_gate:
            vx, vy, wz = self.gate_release_limiter.update(
                msg.linear.x, msg.linear.y, msg.angular.z, now_s
            )
        else:
            vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        self._send_velocity(vx, vy, wz, source="nav2")

    def _motion_is_authorized(self) -> bool:
        if not self.enforce_task_gate:
            return True
        return navigation_motion_is_authorized(
            self._pose_is_fresh(),
            self.last_navigator_status_s > 0.0
            and time.monotonic() - self.last_navigator_status_s
            <= self.navigator_status_timeout_s,
            self.goal_request_s > 0.0
            and time.monotonic() - self.goal_request_s
            >= self.goal_handoff_hold_s,
            self.navigator_following_s > 0.0
            and time.monotonic() - self.navigator_following_s
            >= self.navigator_following_hold_s,
            self.pose.task_state if self.pose is not None else -1,
            self.pose.nav_status if self.pose is not None else NAV_IDLE,
            self.requested_request_id,
            self.requested_goal_id,
            self.navigator_request_id,
            self.navigator_goal_id,
            self.navigator_state,
        )

    def _send_velocity(self, vx: float, vy: float, wz: float, source: str) -> None:
        # Keep ROS body signs on the wire. The STM32 is the only layer that
        # knows motor mounting signs and converts these values to wheel RPM.
        vx_mm = vx * 1000.0
        vy_mm = vy * 1000.0
        wz_cdeg = wz * 18000.0 / math.pi
        vx_mm = max(-self.max_speed_mm_s, min(self.max_speed_mm_s, vx_mm))
        vy_mm = max(-self.max_speed_mm_s, min(self.max_speed_mm_s, vy_mm))
        wz_cdeg = max(-self.max_yaw_cdeg_s, min(self.max_yaw_cdeg_s, wz_cdeg))
        self.last_sent = (vx_mm, vy_mm, wz_cdeg)

        if self.dry_run:
            return

        stamp_cs = int((time.monotonic() - self.started_s) * 100.0) & 0xFFFF
        payload = encode_velocity(vx_mm / 1000.0, vy_mm / 1000.0,
                                  wz_cdeg * math.pi / 18000.0, stamp_cs)
        frame = encode_frame(MSG_VEL_CMD, self.tx_seq, payload)
        if self.transport.send(frame):
            self.tx_seq = (self.tx_seq + 1) & 0xFF
            self.cmd_count += 1
            self.get_logger().debug(
                f"{source}: vx={vx_mm:.0f} vy={vy_mm:.0f} wz={wz_cdeg:.0f} mm/s,cdeg/s"
            )

    def _navigator_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
            request_id = int(status.get("request_id", 0)) & 0xFFFF
            goal_id = int(status.get("goal_id", 0)) & 0xFF
            nav_state = int(status.get("state", 0))
            plan_ready = bool(status.get("plan_ready", False))
            if not 0 <= nav_state <= 4:
                raise ValueError(f"invalid navigation state {nav_state}")
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Bad navigator status: {exc}")
            return

        if (
            request_id == self.requested_request_id
            and goal_id == self.requested_goal_id
        ):
            received_s = time.monotonic()
            was_following = (
                self.navigator_request_id == request_id
                and self.navigator_goal_id == goal_id
                and self.navigator_state == NAV_FOLLOWING
                and self.last_navigator_status_s > 0.0
                and received_s - self.last_navigator_status_s
                <= self.navigator_status_timeout_s
            )
            self.navigator_request_id = request_id
            self.navigator_goal_id = goal_id
            effective_nav_state = (
                NAV_WAIT_PATH
                if nav_state == NAV_FOLLOWING and not plan_ready
                else nav_state
            )
            self.navigator_state = effective_nav_state
            self.navigator_plan_ready = plan_ready
            self.last_navigator_status_s = received_s
            if effective_nav_state == NAV_FOLLOWING:
                if not was_following:
                    self.navigator_following_s = self.last_navigator_status_s
                    self.stopped = True
                    self.gate_release_limiter.reset(received_s)
                    self._send_velocity(
                        0.0, 0.0, 0.0, source="navigator_following_hold"
                    )
            else:
                self.navigator_following_s = 0.0
                if not self.nurse_scan_soft_stop:
                    self.stopped = True
                    self.gate_release_limiter.reset(received_s)
                    self._send_velocity(0.0, 0.0, 0.0, source="navigator_state")
            payload = encode_nav_status(request_id, goal_id, effective_nav_state)
            frame = encode_frame(MSG_NAV_STATUS, self.tx_seq, payload)
            if self.transport.send(frame):
                self.tx_seq = (self.tx_seq + 1) & 0xFF
        else:
            self.get_logger().warn(
                "Ignored stale navigator status for "
                f"request={request_id} goal={goal_id}",
                throttle_duration_sec=1.0,
            )

    def _command_watchdog(self) -> None:
        if self.last_cmd_s == 0.0:
            return
        silent_for = time.monotonic() - self.last_cmd_s
        if silent_for <= self.cmd_timeout_s or self.stopped:
            return
        self.stopped = True
        self.last_safe_cmd = (0.0, 0.0, 0.0)
        self.gate_release_limiter.reset(time.monotonic())
        self._send_velocity(0.0, 0.0, 0.0, source="watchdog")
        self.get_logger().warn(
            f"No {self.cmd_vel_topic} for {silent_for:.2f}s - sending stop",
            throttle_duration_sec=1.0,
        )

    def _heartbeat(self) -> None:
        self.heartbeat_counter = (self.heartbeat_counter + 1) & 0xFFFFFFFF
        frame = encode_frame(
            MSG_HEARTBEAT, self.tx_seq, encode_heartbeat(self.heartbeat_counter)
        )
        if self.transport.send(frame):
            self.tx_seq = (self.tx_seq + 1) & 0xFF

    def _publish_status(self) -> None:
        status = {
            "serial": self.transport.connected,
            "pose": self._pose_is_fresh(),
            "wheel_odom": self._wheel_odom_is_fresh(),
            "odom_source": (
                "ops_hwt_wheel_ekf"
                if self._wheel_odom_is_fresh()
                else "ops_hwt_fallback"
            ),
            "mission_epoch": self.mission_epoch,
            "cmd_count": self.cmd_count,
            "stopped": self.stopped,
            "dry_run": self.dry_run,
            "task_gate": self.enforce_task_gate,
            "motion_authorized": self._motion_is_authorized(),
            "nurse_scan_soft_stop": self.nurse_scan_soft_stop,
            "gate_release_limiter": {
                "enabled": self.enforce_task_gate
                and self.gate_release_limiter.max_wheel_accel_m_s2 > 0.0,
                "active": self.enforce_task_gate
                and self.gate_release_limiter.active,
                "limiting": self.gate_release_limiter.limiting,
                "wheel_accel_m_s2": self.gate_release_limiter.max_wheel_accel_m_s2,
                "yaw_radius_m": self.gate_release_limiter.yaw_radius_m,
                "rearm_drop_m_s": (
                    self.gate_release_limiter.rearm_drop_m_s
                ),
                "output": {
                    "vx": round(self.gate_release_limiter.output[0], 3),
                    "vy": round(self.gate_release_limiter.output[1], 3),
                    "wz": round(self.gate_release_limiter.output[2], 3),
                },
            },
            "last_safe_cmd": {
                "vx": round(self.last_safe_cmd[0], 3),
                "vy": round(self.last_safe_cmd[1], 3),
                "wz": round(self.last_safe_cmd[2], 3),
            },
            "goal_gate": {
                "requested_request_id": self.requested_request_id,
                "requested_goal_id": self.requested_goal_id,
                "navigator_request_id": self.navigator_request_id,
                "navigator_goal_id": self.navigator_goal_id,
                "navigator_state": self.navigator_state,
                "plan_ready": self.navigator_plan_ready,
                "navigator_fresh": self.last_navigator_status_s > 0.0
                and time.monotonic() - self.last_navigator_status_s
                <= self.navigator_status_timeout_s,
                "handoff_ready": self.goal_request_s > 0.0
                and time.monotonic() - self.goal_request_s
                >= self.goal_handoff_hold_s,
                "navigator_following_ready": self.navigator_following_s > 0.0
                and time.monotonic() - self.navigator_following_s
                >= self.navigator_following_hold_s,
            },
            "ops_offset_mm": {
                "x": round(self.calibrator.offset_x_mm, 1),
                "y": round(self.calibrator.offset_y_mm, 1),
            },
            "last_cmd": {
                "vx_mm_s": round(self.last_sent[0], 1),
                "vy_mm_s": round(self.last_sent[1], 1),
                "w_cdeg_s": round(self.last_sent[2], 1),
            },
        }
        if self.pose is not None:
            waiting_for_start = self.pose.task_state in START_WAIT_TASK_STATES
            path_ready = (
                waiting_for_start and self.pose.nav_status == NAV_FOLLOWING
            )
            status["start_gate"] = {
                "waiting_for_button": waiting_for_start,
                "path_ready": path_ready,
                "can_start": path_ready,
            }
            status["stm32"] = {
                "x_mm": self.pose.x_mm,
                "y_mm": self.pose.y_mm,
                "yaw_cdeg": self.pose.yaw_cdeg,
                "task_state": self.pose.task_state,
                "nav_status": self.pose.nav_status,
            }
        else:
            status["start_gate"] = {
                "waiting_for_button": False,
                "path_ready": False,
                "can_start": False,
            }
        if self.stp23l is not None:
            status["stp23l"] = {
                "a_right_mm": self.stp23l.a_mm,
                "b_front_mm": self.stp23l.b_mm,
                "c_left_mm": self.stp23l.c_mm,
                "valid_mask": self.stp23l.valid_mask,
            }
        if self.wheel_odom is not None:
            status["wheel"] = {
                "forward_mm_s": self.wheel_odom.forward_mm_s,
                "left_mm_s": self.wheel_odom.left_mm_s,
                "yaw_ccw_cdeg_s": self.wheel_odom.yaw_ccw_cdeg_s,
                "online_mask": self.wheel_odom.online_mask,
                "stamp_cs": self.wheel_odom.stamp_cs,
            }
        if self.wheel_diagnostics is not None:
            status["wheel_diagnostics"] = asdict(self.wheel_diagnostics)
        status["scanner_transport"] = {
            "pending": self.pending_scan is not None,
            "attempts": self.pending_scan["attempts"] if self.pending_scan else 0,
            "value": self.pending_scan["value"] if self.pending_scan else None,
            "last_ack": self.last_scan_ack,
        }
        status["tts"] = {
            "playing": self.tts_future is not None,
            "keepalive": self.tts_player.keepalive_active,
            "last_status": self.last_tts_status,
            "bed1_audio_file": self.tts_player.audio_files[1],
            "bed3_audio_file": self.tts_player.audio_files[3],
            "volume_percent": self.tts_player.volume_percent,
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(message)

    def destroy_node(self):
        self.tts_executor.shutdown(wait=False, cancel_futures=True)
        self.tts_player.stop_keepalive()
        self.transport.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stm32Bridge()
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
