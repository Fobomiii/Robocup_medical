#!/usr/bin/env python3
"""Compare OPS displacement with robust temporal Mid360 scan matching."""

from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool, String

from .lidar_odometry_core import (
    Pose2D,
    dilate_mask,
    interpolate_pose,
    points_in_grid_mask,
    relative_motion,
    transform_points,
    translation_icp,
    voxel_downsample,
)


def _yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)


def _stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


class LidarOdometryGuard(Node):
    def __init__(self) -> None:
        super().__init__("lidar_odometry_guard")

        defaults = {
            "enabled": True,
            "monitor_only": True,
            "cloud_topic": "/livox/lidar_filtered",
            "odom_topic": "/odom",
            "map_topic": "/map",
            "use_static_map_filter": False,
            "process_period_s": 0.50,
            "max_window_s": 1.0,
            "max_odom_sync_error_s": 0.15,
            "min_range": 0.35,
            "max_range": 5.0,
            "min_height": 0.08,
            "max_height": 0.60,
            "voxel_size": 0.08,
            "static_map_tolerance": 0.30,
            "occupied_threshold": 65,
            "max_points": 600,
            "min_reference_points": 60,
            "max_correspondence_distance": 0.20,
            "max_iterations": 8,
            "trim_fraction": 0.75,
            "min_inlier_ratio": 0.45,
            "max_rmse": 0.09,
            "min_geometry_ratio": 0.02,
            "min_spatial_extent": 1.0,
            "slip_translation_threshold": 0.08,
            "required_bad_windows": 3,
            "required_good_windows": 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value

        self.enabled = bool(get("enabled"))
        self.monitor_only = bool(get("monitor_only"))
        self.cloud_topic = str(get("cloud_topic"))
        self.odom_topic = str(get("odom_topic"))
        self.map_topic = str(get("map_topic"))
        self.use_static_map_filter = bool(get("use_static_map_filter"))
        self.process_period_s = float(get("process_period_s"))
        self.max_window_s = float(get("max_window_s"))
        self.max_odom_sync_error_s = float(get("max_odom_sync_error_s"))
        self.min_range = float(get("min_range"))
        self.max_range = float(get("max_range"))
        self.min_height = float(get("min_height"))
        self.max_height = float(get("max_height"))
        self.voxel_size = float(get("voxel_size"))
        self.static_map_tolerance = float(get("static_map_tolerance"))
        self.occupied_threshold = int(get("occupied_threshold"))
        self.max_points = int(get("max_points"))
        self.min_reference_points = int(get("min_reference_points"))
        self.max_correspondence_distance = float(get("max_correspondence_distance"))
        self.max_iterations = int(get("max_iterations"))
        self.trim_fraction = float(get("trim_fraction"))
        self.min_inlier_ratio = float(get("min_inlier_ratio"))
        self.max_rmse = float(get("max_rmse"))
        self.min_geometry_ratio = float(get("min_geometry_ratio"))
        self.min_spatial_extent = float(get("min_spatial_extent"))
        self.slip_translation_threshold = float(get("slip_translation_threshold"))
        self.required_bad_windows = int(get("required_bad_windows"))
        self.required_good_windows = int(get("required_good_windows"))

        if (
            self.process_period_s <= 0.0
            or self.max_window_s < self.process_period_s
            or self.voxel_size <= 0.0
        ):
            raise ValueError("invalid process period, window, or voxel size")
        if not 0.0 < self.trim_fraction <= 1.0:
            raise ValueError("trim_fraction must be in (0, 1]")
        if self.min_reference_points < 3 or self.max_points < self.min_reference_points:
            raise ValueError("invalid reference point limits")
        if not self.monitor_only:
            raise ValueError("only monitor_only mode is implemented and validated")

        self.odom_history = deque(maxlen=250)
        self.static_support = None
        self.static_origin = (0.0, 0.0)
        self.static_resolution = 0.0
        self.static_cell_count = 0
        self.map_frame = ""
        self.previous_pose = None
        self.previous_points = None
        self.previous_cloud_stamp_ns = 0
        self.last_process_s = 0.0
        self.bad_windows = 0
        self.good_windows = 0
        self.slip_suspected = False
        self.last_status = {
            "enabled": self.enabled,
            "monitor_only": self.monitor_only,
            "valid": False,
            "reason": "waiting_for_data" if self.enabled else "disabled",
        }

        self.status_pub = self.create_publisher(
            String, "/medical_nav/lidar_odometry_status", 10
        )
        self.slip_pub = self.create_publisher(
            Bool, "/medical_nav/ops_slip_suspected", 10
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, self.odom_topic, self._odom, 50)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map, map_qos)
        self.create_subscription(
            PointCloud2, self.cloud_topic, self._cloud, qos_profile_sensor_data
        )
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"Lidar odometry guard {'enabled' if self.enabled else 'disabled'} | "
            f"monitor_only={self.monitor_only} | period={self.process_period_s:.2f}s | "
            f"reference={'static_map' if self.use_static_map_filter else 'temporal'}"
        )

    def _odom(self, message: Odometry) -> None:
        stamp_ns = _stamp_ns(message)
        if stamp_ns <= 0:
            return
        position = message.pose.pose.position
        self.odom_history.append(
            Pose2D(stamp_ns, position.x, position.y, _yaw_from_odometry(message))
        )

    def _map(self, message: OccupancyGrid) -> None:
        info = message.info
        if info.resolution <= 0.0 or info.width == 0 or info.height == 0:
            return
        # The generated competition map has zero origin yaw. Reject a rotated
        # map rather than silently applying the wrong static-point mask.
        q = info.origin.orientation
        map_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        if abs(map_yaw) > 1e-4:
            self.static_support = None
            self.last_status.update(valid=False, reason="rotated_map_not_supported")
            return
        occupancy = np.asarray(message.data, dtype=np.int16).reshape(
            (info.height, info.width)
        )
        occupied = occupancy >= self.occupied_threshold
        radius_cells = int(math.ceil(self.static_map_tolerance / info.resolution))
        self.static_support = dilate_mask(occupied, radius_cells)
        self.static_origin = (info.origin.position.x, info.origin.position.y)
        self.static_resolution = float(info.resolution)
        self.static_cell_count = int(np.count_nonzero(occupied))
        self.map_frame = message.header.frame_id
        self.last_status["static_cells"] = self.static_cell_count

    def _cloud(self, message: PointCloud2) -> None:
        configured_enabled = bool(self.get_parameter("enabled").value)
        if configured_enabled != self.enabled:
            self.enabled = configured_enabled
            self.previous_pose = None
            self.previous_points = None
            self.previous_cloud_stamp_ns = 0
            self.bad_windows = 0
            self.good_windows = 0
            self.slip_suspected = False
            self._invalidate("waiting_for_data" if self.enabled else "disabled")
            self.get_logger().info(
                f"Lidar odometry guard {'enabled' if self.enabled else 'disabled'}"
            )
        if not self.enabled:
            return
        now_s = time.monotonic()
        if now_s - self.last_process_s < self.process_period_s:
            return
        self.last_process_s = now_s

        cloud_stamp_ns = _stamp_ns(message)
        pose = interpolate_pose(
            list(self.odom_history), cloud_stamp_ns, self.max_odom_sync_error_s
        )
        if pose is None:
            self._invalidate("odom_not_time_aligned")
            return
        if (
            self.use_static_map_filter
            and (self.static_support is None or self.static_cell_count == 0)
        ):
            self._invalidate("static_map_unavailable")
            return
        if message.header.frame_id.lstrip("/") != "base_link":
            self._invalidate("cloud_not_in_base_link")
            return

        points = pc2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        if points.size == 0:
            self._invalidate("empty_cloud")
            return
        xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(
            np.float64, copy=False
        )
        ranges_sq = xyz[:, 0] ** 2 + xyz[:, 1] ** 2
        keep = (
            (ranges_sq >= self.min_range ** 2)
            & (ranges_sq <= self.max_range ** 2)
            & (xyz[:, 2] >= self.min_height)
            & (xyz[:, 2] <= self.max_height)
        )
        points_base = voxel_downsample(xyz[keep, :2], self.voxel_size)
        if len(points_base) == 0:
            self._invalidate("no_points_after_filter")
            return

        if self.use_static_map_filter:
            points_map = transform_points(
                points_base, np.array([pose.x, pose.y]), pose.yaw
            )
            static_mask = points_in_grid_mask(
                points_map,
                self.static_support,
                self.static_origin[0],
                self.static_origin[1],
                self.static_resolution,
            )
            points_base = points_base[static_mask]
        if len(points_base) > self.max_points:
            # Deterministic even sampling retains the entire visible geometry.
            indices = np.linspace(
                0, len(points_base) - 1, self.max_points, dtype=np.int64
            )
            points_base = points_base[indices]
        if len(points_base) < self.min_reference_points:
            self.previous_pose = None
            self.previous_points = None
            self._invalidate("too_few_static_points", points=len(points_base))
            return

        if self.previous_pose is None or self.previous_points is None:
            self._set_reference(pose, points_base, cloud_stamp_ns)
            self._invalidate("reference_initialized", points=len(points_base))
            return

        duration_s = (cloud_stamp_ns - self.previous_cloud_stamp_ns) * 1e-9
        if duration_s <= 0.0 or duration_s > self.max_window_s:
            self._set_reference(pose, points_base, cloud_stamp_ns)
            self._invalidate(
                "scan_window_reset", points=len(points_base), dt_s=round(duration_s, 3)
            )
            return

        ops_translation, relative_yaw = relative_motion(self.previous_pose, pose)
        result = translation_icp(
            points_base,
            self.previous_points,
            relative_yaw,
            ops_translation,
            self.max_correspondence_distance,
            self.max_iterations,
            trim_fraction=self.trim_fraction,
        )
        self._set_reference(pose, points_base, cloud_stamp_ns)
        if result is None:
            self._invalidate("scan_match_failed", points=len(points_base))
            return

        quality_reason = "ok"
        if result.inlier_count < self.min_reference_points:
            quality_reason = "too_few_inliers"
        elif result.inlier_ratio < self.min_inlier_ratio:
            quality_reason = "low_inlier_ratio"
        elif result.rmse > self.max_rmse:
            quality_reason = "high_rmse"
        elif result.geometry_ratio < self.min_geometry_ratio:
            quality_reason = "degenerate_geometry"
        elif result.spatial_extent < self.min_spatial_extent:
            quality_reason = "insufficient_spatial_extent"

        difference = result.translation - ops_translation
        difference_m = float(np.linalg.norm(difference))
        valid = quality_reason == "ok"
        if valid:
            previously_suspected = self.slip_suspected
            if difference_m >= self.slip_translation_threshold:
                self.bad_windows += 1
                self.good_windows = 0
            else:
                self.good_windows += 1
                self.bad_windows = 0
            if self.bad_windows >= self.required_bad_windows:
                self.slip_suspected = True
            elif self.good_windows >= self.required_good_windows:
                self.slip_suspected = False
            if self.slip_suspected and not previously_suspected:
                self.get_logger().warning(
                    "OPS slip suspected: OPS/Livox displacement differs by "
                    f"{difference_m:.3f} m for {self.bad_windows} windows"
                )
            elif previously_suspected and not self.slip_suspected:
                self.get_logger().info(
                    "OPS/Livox displacement agreement recovered"
                )

        self.last_status = {
            "enabled": True,
            "monitor_only": self.monitor_only,
            "valid": valid,
            "reason": quality_reason,
            "dt_s": round(duration_s, 3),
            "points": len(points_base),
            "inliers": result.inlier_count,
            "inlier_ratio": round(result.inlier_ratio, 4),
            "rmse_m": round(result.rmse, 4),
            "geometry_ratio": round(result.geometry_ratio, 4),
            "spatial_extent_m": round(result.spatial_extent, 4),
            "iterations": result.iterations,
            "ops_delta_m": [round(float(value), 4) for value in ops_translation],
            "lidar_delta_m": [
                round(float(value), 4) for value in result.translation
            ],
            "difference_xy_m": [round(float(value), 4) for value in difference],
            "difference_m": round(difference_m, 4),
            "relative_yaw_deg": round(math.degrees(relative_yaw), 3),
            "bad_windows": self.bad_windows,
            "good_windows": self.good_windows,
            "slip_suspected": self.slip_suspected,
            "reference_mode": (
                "static_map" if self.use_static_map_filter else "temporal"
            ),
            "static_cells": self.static_cell_count,
        }

    def _set_reference(
        self, pose: Pose2D, points: np.ndarray, cloud_stamp_ns: int
    ) -> None:
        self.previous_pose = pose
        self.previous_points = points
        self.previous_cloud_stamp_ns = cloud_stamp_ns

    def _invalidate(self, reason: str, **extra) -> None:
        self.last_status = {
            "enabled": self.enabled,
            "monitor_only": self.monitor_only,
            "valid": False,
            "reason": reason,
            "slip_suspected": self.slip_suspected,
            "bad_windows": self.bad_windows,
            **extra,
        }

    def _publish_status(self) -> None:
        status = String()
        status.data = json.dumps(self.last_status, ensure_ascii=False)
        self.status_pub.publish(status)
        slip = Bool()
        slip.data = self.slip_suspected
        self.slip_pub.publish(slip)


def main(args=None):
    rclpy.init(args=args)
    node = LidarOdometryGuard()
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
