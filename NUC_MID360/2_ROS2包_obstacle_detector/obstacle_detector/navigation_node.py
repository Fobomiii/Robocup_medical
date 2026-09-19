#!/usr/bin/env python3
"""Medical robot perception, path planning, and STM32 serial bridge."""

from collections import deque
import json
import math
import os
import struct
import threading
import time
from typing import Callable, Deque

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
import serial
import serial.tools.list_ports
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .fixed_routes import FixedRouteMap, Waypoint
from .navigation_core import GridPlanner, ObstacleMapper
from .nav_protocol import (
    Frame,
    FrameParser,
    GOAL_NONE,
    MSG_GOAL_REQUEST,
    MSG_HEARTBEAT,
    MSG_PATH_CANCEL,
    MSG_POSE,
    decode_goal_request,
    decode_pose,
    encode_frame,
    encode_heartbeat,
    make_path_frames,
)


PING_FRAME = bytes((0xA5, 0x5A, 0x01, 0x00))
PONG_FRAME = bytes((0xA5, 0x5A, 0x01, 0x01))


def parse_cloud(msg: PointCloud2) -> np.ndarray:
    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    array = np.array(list(points))
    if len(array) == 0:
        return np.empty((0, 3), dtype=np.float32)
    if array.dtype.names:
        return np.column_stack([array["x"], array["y"], array["z"]]).astype(np.float32)
    return array.reshape(-1, 3).astype(np.float32)


def yaw_to_quaternion(yaw_clockwise_deg: float):
    # ROS yaw is counter-clockwise; the robot field convention is clockwise.
    half = math.radians(-yaw_clockwise_deg) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class SerialTransport:
    def __init__(
        self,
        port: str,
        baud: int,
        on_frame: Callable[[Frame], None],
        logger,
    ) -> None:
        self.port = port
        self.baud = baud
        self.on_frame = on_frame
        self.logger = logger
        self.parser = FrameParser()
        self.serial = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ping_index = 0
        self.connected = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        with self.lock:
            if self.serial is not None:
                try:
                    self.serial.close()
                except serial.SerialException:
                    pass
                self.serial = None

    def send(self, data: bytes) -> bool:
        try:
            with self.lock:
                if self.serial is None or not self.serial.is_open:
                    return False
                self.serial.write(data)
            return True
        except serial.SerialException as exc:
            self.logger.error(f"Serial write failed: {exc}")
            self._disconnect()
            return False

    def _disconnect(self) -> None:
        with self.lock:
            if self.serial is not None:
                try:
                    self.serial.close()
                except serial.SerialException:
                    pass
            self.serial = None
            self.connected = False

    def _open(self) -> bool:
        try:
            connection = serial.serial_for_url(
                self.port,
                self.baud,
                timeout=0.02,
                write_timeout=0.1,
            )
            with self.lock:
                self.serial = connection
                self.connected = True
            self.logger.info(f"Serial {self.port}@{self.baud} opened")
            return True
        except serial.SerialException:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.logger.warn(f"Serial {self.port} unavailable; detected ports: {ports}")
            return False

    def _feed_handshake(self, value: int) -> None:
        if value == PING_FRAME[self.ping_index]:
            self.ping_index += 1
            if self.ping_index == len(PING_FRAME):
                self.ping_index = 0
                self.send(PONG_FRAME)
        elif value == PING_FRAME[0]:
            self.ping_index = 1
        else:
            self.ping_index = 0

    def _run(self) -> None:
        retry_at = 0.0
        while not self.stop_event.is_set():
            if not self.connected:
                if time.monotonic() < retry_at:
                    time.sleep(0.05)
                    continue
                if not self._open():
                    retry_at = time.monotonic() + 2.0
                    continue

            try:
                with self.lock:
                    connection = self.serial
                data = connection.read(128) if connection is not None else b""
            except serial.SerialException as exc:
                self.logger.error(f"Serial read failed: {exc}")
                self._disconnect()
                retry_at = time.monotonic() + 1.0
                continue

            if not data:
                continue
            for value in data:
                self._feed_handshake(value)
            for frame in self.parser.feed(data):
                self.on_frame(frame)


class MedicalNavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("medical_navigation")
        package_share = get_package_share_directory("obstacle_detector")
        default_map = os.path.join(package_share, "config", "field_map.yaml")

        self.declare_parameter("topic", "/livox/lidar")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("map_config", default_map)
        self.declare_parameter("planner_enabled", False)
        self.declare_parameter("auto_replan", True)
        self.declare_parameter("pose_timeout_s", 0.50)
        self.declare_parameter("point_stride", 2)
        self.declare_parameter("range_min", 0.25)
        self.declare_parameter("range_max", 6.50)
        self.declare_parameter("z_min", -0.10)
        self.declare_parameter("z_max", 0.55)
        self.declare_parameter("lidar_forward_mm", 0.0)
        self.declare_parameter("lidar_right_mm", 0.0)
        self.declare_parameter("lidar_inverted", False)
        self.declare_parameter("fixture_reject_margin_mm", 120.0)
        self.declare_parameter("field_reject_margin_mm", 80.0)
        self.declare_parameter("cluster_cell_mm", 80.0)
        self.declare_parameter("cluster_tolerance_mm", 180.0)
        self.declare_parameter("cluster_min_points", 5)
        self.declare_parameter("cluster_max_span_mm", 650.0)
        self.declare_parameter("obstacle_radius_min_mm", 120.0)
        self.declare_parameter("obstacle_radius_max_mm", 350.0)
        self.declare_parameter("track_match_mm", 350.0)
        self.declare_parameter("confirm_hits", 3)
        self.declare_parameter("candidate_ttl_s", 1.0)
        self.declare_parameter("confirmed_ttl_s", 30.0)
        self.declare_parameter("max_obstacles", 5)
        self.declare_parameter("map_change_mm", 80.0)
        self.declare_parameter("grid_resolution_mm", 100.0)
        self.declare_parameter("inflation_mm", 360.0)
        self.declare_parameter("cruise_speed_mm_s", 120.0)
        self.declare_parameter("corner_speed_mm_s", 90.0)
        self.declare_parameter("replan_cooldown_s", 1.20)

        self.frame_id = "field"
        self.route_map = FixedRouteMap(self.get_parameter("map_config").value)
        self.robot_pose = None
        self.current_path = []
        self.current_path_id = 0
        self.last_request = None
        self.active_request_id = None
        self.active_goal_id = GOAL_NONE
        self.planner_state = "idle"
        self.replan_count = 0
        self.replan_pending = False
        self.last_replan_s = 0.0
        self.last_pose_s = 0.0
        self.last_plan_warning_s = 0.0
        self.tx_seq = 0
        self.heartbeat_counter = 0
        self.rx_queue: Deque[Frame] = deque()
        self.rx_lock = threading.Lock()

        parameter = lambda name: self.get_parameter(name).value
        self.planner_enabled = bool(parameter("planner_enabled"))
        self.auto_replan = bool(parameter("auto_replan"))
        self.pose_timeout_s = float(parameter("pose_timeout_s"))
        self.point_stride = max(1, int(parameter("point_stride")))
        self.replan_cooldown_s = float(parameter("replan_cooldown_s"))
        self.mapper = ObstacleMapper(
            self.route_map.raw["field"],
            self.route_map.raw.get("fixtures", []),
            range_min_m=float(parameter("range_min")),
            range_max_m=float(parameter("range_max")),
            z_min_m=float(parameter("z_min")),
            z_max_m=float(parameter("z_max")),
            lidar_forward_mm=float(parameter("lidar_forward_mm")),
            lidar_right_mm=float(parameter("lidar_right_mm")),
            lidar_inverted=bool(parameter("lidar_inverted")),
            fixture_reject_margin_mm=float(parameter("fixture_reject_margin_mm")),
            field_reject_margin_mm=float(parameter("field_reject_margin_mm")),
            cluster_cell_mm=float(parameter("cluster_cell_mm")),
            cluster_tolerance_mm=float(parameter("cluster_tolerance_mm")),
            cluster_min_points=int(parameter("cluster_min_points")),
            cluster_max_span_mm=float(parameter("cluster_max_span_mm")),
            obstacle_radius_min_mm=float(parameter("obstacle_radius_min_mm")),
            obstacle_radius_max_mm=float(parameter("obstacle_radius_max_mm")),
            track_match_mm=float(parameter("track_match_mm")),
            confirm_hits=int(parameter("confirm_hits")),
            candidate_ttl_s=float(parameter("candidate_ttl_s")),
            confirmed_ttl_s=float(parameter("confirmed_ttl_s")),
            max_obstacles=int(parameter("max_obstacles")),
            map_change_mm=float(parameter("map_change_mm")),
        )
        self.planner = GridPlanner(
            self.route_map.raw["field"],
            self.route_map.raw.get("fixtures", []),
            resolution_mm=float(parameter("grid_resolution_mm")),
            inflation_mm=float(parameter("inflation_mm")),
            cruise_speed_mm_s=float(parameter("cruise_speed_mm_s")),
            corner_speed_mm_s=float(parameter("corner_speed_mm_s")),
        )

        self.pose_pub = self.create_publisher(PoseStamped, "/medical_nav/robot_pose", 10)
        self.path_pub = self.create_publisher(Path, "/medical_nav/path", 10)
        self.obstacle_pub = self.create_publisher(PoseArray, "/medical_nav/obstacles", 10)
        self.status_pub = self.create_publisher(String, "/medical_nav/status", 10)
        self._field_markers_cleared = False
        # Transient local so RViz opened later still receives the current map
        # and field geometry instead of waiting for the next publish.
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, "/medical_nav/costmap", latched
        )
        self.field_marker_pub = self.create_publisher(
            MarkerArray, "/medical_nav/field_markers", latched
        )
        self.obstacle_marker_pub = self.create_publisher(
            MarkerArray, "/medical_nav/obstacle_markers", 10
        )

        topic = self.get_parameter("topic").value
        self.cloud_sub = self.create_subscription(PointCloud2, topic, self._cloud_callback, 10)

        self.transport = SerialTransport(
            str(self.get_parameter("serial_port").value),
            int(self.get_parameter("baud_rate").value),
            self._queue_frame,
            self.get_logger(),
        )
        self.transport.start()
        self.frame_timer = self.create_timer(0.02, self._process_frames)
        self.heartbeat_timer = self.create_timer(0.10, self._send_heartbeat)
        self.status_timer = self.create_timer(0.50, self._publish_status)
        self.planner_timer = self.create_timer(0.20, self._planner_tick)
        self.costmap_timer = self.create_timer(0.50, self._publish_costmap)
        self.obstacle_marker_timer = self.create_timer(0.50, self._publish_obstacle_markers)
        self.field_marker_timer = self.create_timer(2.00, self._publish_field_markers)

        self.get_logger().info(
            f"Medical navigation ready | planner={'A*' if self.planner_enabled else 'fixed'} "
            f"| map={self.route_map.config_path} | cloud={topic}"
        )

    def _queue_frame(self, frame: Frame) -> None:
        with self.rx_lock:
            self.rx_queue.append(frame)

    def _process_frames(self) -> None:
        with self.rx_lock:
            frames = list(self.rx_queue)
            self.rx_queue.clear()
        for frame in frames:
            try:
                if frame.msg_type == MSG_POSE:
                    self._handle_pose(frame)
                elif frame.msg_type == MSG_GOAL_REQUEST:
                    self._handle_goal(frame)
            except (ValueError, struct.error) as exc:
                self.get_logger().warn(f"Rejected frame type 0x{frame.msg_type:02X}: {exc}")

    def _handle_pose(self, frame: Frame) -> None:
        self.robot_pose = decode_pose(frame.payload)
        self.last_pose_s = time.monotonic()
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.pose.position.x = self.robot_pose.x_mm / 1000.0
        message.pose.position.y = self.robot_pose.y_mm / 1000.0
        qx, qy, qz, qw = yaw_to_quaternion(self.robot_pose.yaw_cdeg / 100.0)
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        self.pose_pub.publish(message)

    def _handle_goal(self, frame: Frame) -> None:
        request = decode_goal_request(frame.payload)
        if request.goal_id == GOAL_NONE:
            return

        if request.goal_id not in self.route_map.goals_by_id:
            raise ValueError(f"unknown goal id: {request.goal_id}")

        self.active_request_id = request.request_id
        self.active_goal_id = request.goal_id

        if self.last_request == (request.request_id, request.goal_id) and self.current_path:
            self._send_current_path(request.request_id, request.goal_id)
            return

        if not self._pose_is_fresh():
            self.planner_state = "waiting_pose"
            self.replan_pending = True
            return
        self._plan_for_active(replanned=False)

    def _plan_for_active(self, replanned: bool) -> bool:
        if self.active_request_id is None or self.active_goal_id == GOAL_NONE:
            return False
        if not self._pose_is_fresh():
            self.planner_state = "waiting_pose"
            return False

        x_mm = float(self.robot_pose.x_mm)
        y_mm = float(self.robot_pose.y_mm)
        source, goal, fixed_waypoints = self.route_map.path_for(
            self.active_goal_id, x_mm, y_mm
        )
        if self.planner_enabled:
            waypoints = self.planner.plan(x_mm, y_mm, goal, self.mapper.obstacles())
            route_kind = "A*"
        else:
            waypoints = fixed_waypoints
            route_kind = "fixed"

        if not waypoints:
            self.planner_state = "no_path"
            self._cancel_active_path()
            now = time.monotonic()
            if now - self.last_plan_warning_s >= 2.0:
                self.get_logger().error(
                    f"No collision-free path {source}->{goal.name}; robot held for retry"
                )
                self.last_plan_warning_s = now
            return False

        self.current_path_id = (self.current_path_id + 1) & 0xFFFF
        if self.current_path_id == 0:
            self.current_path_id = 1
        self.current_path = waypoints
        self.last_request = (self.active_request_id, self.active_goal_id)
        self._send_current_path(self.active_request_id, self.active_goal_id)
        self._publish_path(waypoints)
        self.last_replan_s = time.monotonic()
        self.replan_pending = False
        if replanned:
            self.replan_count += 1
        self.planner_state = "replanned" if replanned else ("planned" if self.planner_enabled else "fixed")
        self.get_logger().info(
            f"{route_kind} route {source}->{goal.name}: path={self.current_path_id}, "
            f"request={self.active_request_id}, points={len(waypoints)}, "
            f"obstacles={len(self.mapper.obstacles())}"
        )
        return True

    def _cancel_active_path(self) -> None:
        frame = encode_frame(MSG_PATH_CANCEL, self.tx_seq)
        if self.transport.send(frame):
            self.tx_seq = (self.tx_seq + 1) & 0xFF
        self.current_path = []
        self.last_request = None
        self._publish_path([])

    def _pose_is_fresh(self) -> bool:
        return (
            self.robot_pose is not None
            and time.monotonic() - self.last_pose_s <= self.pose_timeout_s
        )

    def _planner_tick(self) -> None:
        if self.active_request_id is None or not self.replan_pending:
            return
        if not self._pose_is_fresh():
            return
        if time.monotonic() - self.last_replan_s < self.replan_cooldown_s:
            return

        if self.current_path and self.planner_enabled and self.auto_replan:
            index = max(0, int(self.robot_pose.waypoint_index))
            remaining = self.current_path[index:]
            blocked = self.planner.path_blocked(
                (self.robot_pose.x_mm, self.robot_pose.y_mm),
                remaining,
                self.mapper.obstacles(),
            )
            if not blocked:
                self.replan_pending = False
                return
            x_mm = float(self.robot_pose.x_mm)
            y_mm = float(self.robot_pose.y_mm)
            goal = self.route_map.goals_by_id[self.active_goal_id]
            trial = self.planner.plan(x_mm, y_mm, goal, self.mapper.obstacles())
            if trial and not self._paths_similar(remaining, trial, 300.0):
                self._cancel_active_path()
                self._plan_for_active(replanned=True)
            else:
                self.replan_pending = False
        elif not self.current_path:
            self._plan_for_active(replanned=False)

    @staticmethod
    def _paths_similar(old_wps, new_wps, threshold_mm: float) -> bool:
        if not old_wps or not new_wps:
            return False
        old_pts = [(w.x_mm, w.y_mm) for w in old_wps]
        new_pts = [(w.x_mm, w.y_mm) for w in new_wps]
        max_dev = 0.0
        for pt in new_pts:
            min_d = min(math.hypot(pt[0] - o[0], pt[1] - o[1]) for o in old_pts)
            max_dev = max(max_dev, min_d)
        return max_dev < threshold_mm

    def _send_current_path(self, request_id: int, goal_id: int) -> None:
        frames = make_path_frames(
            self.current_path_id,
            request_id,
            goal_id,
            self.current_path,
            self.tx_seq,
        )
        for frame in frames:
            self.transport.send(frame)
        self.tx_seq = (self.tx_seq + len(frames)) & 0xFF

    def _publish_path(self, waypoints) -> None:
        now = self.get_clock().now().to_msg()
        message = Path()
        message.header.stamp = now
        message.header.frame_id = self.frame_id
        if waypoints and self.robot_pose is not None:
            start = Waypoint(
                self.robot_pose.x_mm,
                self.robot_pose.y_mm,
                self.robot_pose.yaw_cdeg / 100.0,
                0.0,
            )
            points = [start] + list(waypoints)
        elif waypoints:
            points = list(waypoints)
        else:
            points = []
        for point in points:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = point.x_mm / 1000.0
            pose.pose.position.y = point.y_mm / 1000.0
            qx, qy, qz, qw = yaw_to_quaternion(point.yaw_deg)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _cloud_callback(self, msg: PointCloud2) -> None:
        output = PoseArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.frame_id

        if not self._pose_is_fresh():
            self.obstacle_pub.publish(output)
            return

        points = parse_cloud(msg)
        changed = self.mapper.update(
            points[:: self.point_stride],
            self.robot_pose.x_mm,
            self.robot_pose.y_mm,
            self.robot_pose.yaw_cdeg / 100.0,
            time.monotonic(),
        )
        if changed and self.planner_enabled and self.auto_replan:
            self.replan_pending = True
        for obstacle in self.mapper.obstacles():
            pose = Pose()
            pose.position.x = obstacle.x_mm / 1000.0
            pose.position.y = obstacle.y_mm / 1000.0
            pose.position.z = obstacle.radius_mm / 1000.0
            pose.orientation.w = 1.0
            output.poses.append(pose)
        self.obstacle_pub.publish(output)

    def _publish_costmap(self) -> None:
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.info.resolution = self.mapper.grid_resolution_mm / 1000.0
        message.info.width = self.mapper.grid_w
        message.info.height = self.mapper.grid_h
        message.info.origin.position.x = self.mapper.grid_x_min / 1000.0
        message.info.origin.position.y = self.mapper.grid_y_min / 1000.0
        message.info.origin.orientation.w = 1.0
        message.data = self.mapper.costmap_data().flatten().tolist()
        self.costmap_pub.publish(message)

    def _publish_field_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        field = self.mapper.field
        message = MarkerArray()

        if not self._field_markers_cleared:
            # Wipe anything a previous run of this node left behind, so
            # restarting does not stack a second copy of the field in RViz.
            self._field_markers_cleared = True
            clear = Marker()
            clear.header.stamp = stamp
            clear.header.frame_id = self.frame_id
            clear.action = Marker.DELETEALL
            message.markers.append(clear)

        for index, fixture in enumerate(self.mapper.fixtures, start=1):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.frame_id
            marker.ns = "fixtures"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(fixture["x_mm"]) / 1000.0
            marker.pose.position.y = float(fixture["y_mm"]) / 1000.0
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(fixture["width_mm"]) / 1000.0
            marker.scale.y = float(fixture["height_mm"]) / 1000.0
            marker.scale.z = 0.5
            marker.color.r = 0.20
            marker.color.g = 0.45
            marker.color.b = 1.0
            marker.color.a = 0.65
            message.markers.append(marker)

        boundary = Marker()
        boundary.header.stamp = stamp
        boundary.header.frame_id = self.frame_id
        boundary.ns = "boundary"
        boundary.id = 0
        boundary.type = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.pose.orientation.w = 1.0
        boundary.scale.x = 0.03
        boundary.color.r = 1.0
        boundary.color.g = 1.0
        boundary.color.b = 1.0
        boundary.color.a = 0.9
        x_min = float(field["x_min_mm"]) / 1000.0
        x_max = float(field["x_max_mm"]) / 1000.0
        y_min = float(field["y_min_mm"]) / 1000.0
        y_max = float(field["y_max_mm"]) / 1000.0
        for x_m, y_m in (
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
            (x_min, y_min),
        ):
            point = Point()
            point.x = x_m
            point.y = y_m
            point.z = 0.0
            boundary.points.append(point)
        message.markers.append(boundary)

        self.field_marker_pub.publish(message)

    def _publish_obstacle_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        message = MarkerArray()
        obstacles = self.mapper.obstacles()

        for index, obstacle in enumerate(obstacles, start=1):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.frame_id
            marker.ns = "obstacles"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle.x_mm / 1000.0
            marker.pose.position.y = obstacle.y_mm / 1000.0
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            diameter = max(0.10, obstacle.radius_mm * 2.0 / 1000.0)
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = 0.5
            if self.mapper.obstacle_is_dynamic(index - 1):
                marker.color.r = 1.0
                marker.color.g = 0.85
                marker.color.b = 0.10
            else:
                marker.color.r = 1.0
                marker.color.g = 0.15
                marker.color.b = 0.15
            marker.color.a = 0.75
            marker.lifetime.sec = 1
            message.markers.append(marker)

        # A MarkerArray only adds: a marker dropped from the list would linger
        # in RViz forever, so retire the ids we are no longer publishing.
        previous = getattr(self, "_obstacle_marker_count", 0)
        for index in range(len(obstacles), previous):
            stale = Marker()
            stale.header.stamp = stamp
            stale.header.frame_id = self.frame_id
            stale.ns = "obstacles"
            stale.id = index + 1
            stale.action = Marker.DELETE
            message.markers.append(stale)
        self._obstacle_marker_count = len(obstacles)

        self.obstacle_marker_pub.publish(message)

    def _send_heartbeat(self) -> None:
        self.heartbeat_counter = (self.heartbeat_counter + 1) & 0xFFFFFFFF
        frame = encode_frame(MSG_HEARTBEAT, self.tx_seq, encode_heartbeat(self.heartbeat_counter))
        if self.transport.send(frame):
            self.tx_seq = (self.tx_seq + 1) & 0xFF

    def _publish_status(self) -> None:
        if self.current_path:
            # Default ROS QoS is volatile; republish so a GUI opened later gets
            # the current route without requesting a new navigation goal.
            self._publish_path(self.current_path)
        status = {
            "serial": self.transport.connected,
            "pose": self._pose_is_fresh(),
            "path_id": self.current_path_id,
            "waypoints": len(self.current_path),
            "planner_enabled": self.planner_enabled,
            "planner_state": self.planner_state,
            "replans": self.replan_count,
            "obstacles": len(self.mapper.obstacles()),
            "map_revision": self.mapper.revision,
            "inflation_mm": self.planner.inflation_mm,
        }
        if self.robot_pose is not None:
            status.update(
                {
                    "task_state": self.robot_pose.task_state,
                    "nav_status": self.robot_pose.nav_status,
                    "waypoint_index": self.robot_pose.waypoint_index,
                }
            )
        message = String()
        message.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(message)

    def destroy_node(self):
        self.transport.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MedicalNavigationNode()
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
