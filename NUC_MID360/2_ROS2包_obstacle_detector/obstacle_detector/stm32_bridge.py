#!/usr/bin/env python3
"""STM32 <-> Nav2 bridge.

Outbound (STM32 -> Nav2)
    MSG_POSE becomes odom->base_link plus Odometry. Position comes only from
    OPS9 and yaw only from HWT101CT; twist is differentiated from consecutive
    telemetry samples for the Nav2 controller.

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
from dataclasses import asdict
from typing import Deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .nav_protocol import (
    GOAL_NONE,
    Frame,
    MSG_GOAL_REQUEST,
    MSG_HEARTBEAT,
    MSG_NAV_STATUS,
    MSG_POSE,
    MSG_STP23L,
    MSG_VEL_CMD,
    NAV_FOLLOWING,
    decode_goal_request,
    decode_pose,
    decode_stp23l,
    encode_frame,
    encode_heartbeat,
    encode_nav_status,
    encode_velocity,
)
from .serial_transport import SerialTransport
from .stp23l_calibration import (
    CalibrationConfig,
    OpsRangeCalibrator,
    load_calibration_config,
)


POINT_NAMES = {
    GOAL_NONE: "none",
    1: "home",
    2: "nurse",
    3: "bed1",
    4: "bed3",
}

NAVIGATION_TASK_STATES = {1, 3, 6, 9}


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
        self.declare_parameter("twist_filter_alpha", 0.35)
        # NUC-side linear clamp: 1.00 m/s per body-axis component. The
        # STM32 applies its own independent 2.00 m/s hard cap.
        self.declare_parameter("max_speed_mm_s", 1000.0)
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
        self.dry_run = bool(get("dry_run"))
        self.enforce_task_gate = bool(get("enforce_task_gate"))
        field_config = str(get("field_config"))
        try:
            calibration_config = load_calibration_config(field_config)
        except (OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"STP23L calibration disabled: cannot load {field_config}: {exc}"
            )
            calibration_config = CalibrationConfig(
                False, 155.0, 7, 40.0, 200.0, 8.0, 400.0, ()
            )
        self.calibrator = OpsRangeCalibrator(calibration_config)
        self.sensor_radius_m = calibration_config.sensor_radius_mm / 1000.0

        self.pose = None
        self.previous_ros_pose = None
        self.twist = (0.0, 0.0, 0.0)
        self.last_pose_s = 0.0
        self.last_cmd_s = 0.0
        self.last_sent = (0.0, 0.0, 0.0)
        self.tx_seq = 0
        self.heartbeat_counter = 0
        self.stopped = False
        self.cmd_count = 0
        self.rx_queue: Deque[Frame] = deque()
        self.started_s = time.monotonic()
        self.stp23l = None
        self.last_calibration_event = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/medical_nav/robot_pose", 10)
        self.raw_pose_pub = self.create_publisher(
            PoseStamped, "/medical_nav/ops_raw_pose", 10
        )
        self.status_pub = self.create_publisher(String, "/medical_nav/bridge_status", 10)
        self.goal_pub = self.create_publisher(String, "/medical_nav/goal_request", 10)
        self.stp23l_pub = self.create_publisher(String, "/medical_nav/stp23l", 10)
        self.calibration_pub = self.create_publisher(
            String, "/medical_nav/ops_calibration", 10
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
            except (ValueError, struct.error) as exc:
                self.get_logger().warn(
                    f"Rejected frame 0x{frame.msg_type:02X}: {exc}", throttle_duration_sec=2.0
                )

    def _handle_pose(self, frame: Frame) -> None:
        pose = decode_pose(frame.payload)
        received_s = time.monotonic()
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
        self.pose = pose
        self.last_pose_s = received_s

    def _handle_goal_request(self, frame: Frame) -> None:
        request = decode_goal_request(frame.payload)
        if request.goal_id == GOAL_NONE:
            return
        name = POINT_NAMES.get(request.goal_id, str(request.goal_id))
        message = String()
        message.data = json.dumps(
            {"request_id": request.request_id, "goal_id": request.goal_id, "point": name}
        )
        self.goal_pub.publish(message)
        self.get_logger().info(
            f"STM32 goal request #{request.request_id} -> {name}", throttle_duration_sec=1.0
        )

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
            self.get_logger().info(
                f"STP23L calibrated at {event.bed}: "
                f"OPS offset=({event.offset_x_mm:.1f}, {event.offset_y_mm:.1f}) mm, "
                f"field pose=({event.measured_x_mm:.1f}, {event.measured_y_mm:.1f}) mm"
            )

    # ------------------------------------------------------------------
    # Pose out
    # ------------------------------------------------------------------

    def _pose_is_fresh(self) -> bool:
        return (
            self.pose is not None
            and time.monotonic() - self.last_pose_s <= self.pose_timeout_s
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
        if self._pose_is_fresh():
            self._publish_pose()
        self._command_watchdog()

    def _publish_pose(self) -> None:
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
        self.last_cmd_s = time.monotonic()
        if not self._motion_is_authorized():
            self.stopped = True
            self._send_velocity(0.0, 0.0, 0.0, source="task_gate")
            return
        self.stopped = False
        self._send_velocity(msg.linear.x, msg.linear.y, msg.angular.z, source="nav2")

    def _motion_is_authorized(self) -> bool:
        if not self.enforce_task_gate:
            return True
        return (
            self._pose_is_fresh()
            and self.pose.task_state in NAVIGATION_TASK_STATES
            and self.pose.nav_status == NAV_FOLLOWING
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
            if not 0 <= nav_state <= 4:
                raise ValueError(f"invalid navigation state {nav_state}")
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Bad navigator status: {exc}")
            return

        payload = encode_nav_status(request_id, goal_id, nav_state)
        frame = encode_frame(MSG_NAV_STATUS, self.tx_seq, payload)
        if self.transport.send(frame):
            self.tx_seq = (self.tx_seq + 1) & 0xFF

    def _command_watchdog(self) -> None:
        if self.last_cmd_s == 0.0:
            return
        silent_for = time.monotonic() - self.last_cmd_s
        if silent_for <= self.cmd_timeout_s or self.stopped:
            return
        self.stopped = True
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
            "cmd_count": self.cmd_count,
            "stopped": self.stopped,
            "dry_run": self.dry_run,
            "task_gate": self.enforce_task_gate,
            "motion_authorized": self._motion_is_authorized(),
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
            status["stm32"] = {
                "x_mm": self.pose.x_mm,
                "y_mm": self.pose.y_mm,
                "yaw_cdeg": self.pose.yaw_cdeg,
                "task_state": self.pose.task_state,
                "nav_status": self.pose.nav_status,
            }
        if self.stp23l is not None:
            status["stp23l"] = {
                "a_right_mm": self.stp23l.a_mm,
                "b_front_mm": self.stp23l.b_mm,
                "c_left_mm": self.stp23l.c_mm,
                "valid_mask": self.stp23l.valid_mask,
            }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(message)

    def destroy_node(self):
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
