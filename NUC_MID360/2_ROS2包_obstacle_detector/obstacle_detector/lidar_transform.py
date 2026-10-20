#!/usr/bin/env python3
"""Transform Mid360 clouds and remove chassis plus floor returns."""

import json
import math
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool, Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from .lidar_deskew import MotionSample, deskew_points
from .qos_profiles import low_latency_sensor_qos


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return the 3x3 rotation matrix for a normalized quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("transform quaternion has zero length")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def transform_points(
    xyz: np.ndarray, rotation: np.ndarray, translation: np.ndarray
) -> np.ndarray:
    """Apply a source-to-target rigid transform to an Nx3 point array."""
    return xyz @ rotation.T + translation


def self_filter_mask(
    xyz: np.ndarray, radius: float, min_z: float, max_z: float
) -> np.ndarray:
    """Select points outside the cylindrical volume occupied by the robot."""
    radial_sq = xyz[:, 0] * xyz[:, 0] + xyz[:, 1] * xyz[:, 1]
    inside_height = (xyz[:, 2] >= min_z) & (xyz[:, 2] <= max_z)
    return ~((radial_sq <= radius * radius) & inside_height)


def regular_polygon_vertices(
    radius: float, sides: int, rotation_deg: float = 0.0
) -> np.ndarray:
    """Return counter-clockwise vertices for a centred regular polygon."""
    if radius <= 0.0:
        raise ValueError("polygon radius must be positive")
    if sides < 3:
        raise ValueError("polygon must have at least three sides")
    rotation = math.radians(rotation_deg)
    angles = rotation + np.arange(sides, dtype=np.float32) * (2.0 * math.pi / sides)
    return np.column_stack((radius * np.cos(angles), radius * np.sin(angles)))


def polygon_self_filter_mask(
    xyz: np.ndarray, radius: float, sides: int, rotation_deg: float,
    min_z: float, max_z: float,
) -> np.ndarray:
    """Select points outside a regular-polygon chassis volume."""
    if len(xyz) == 0:
        return np.ones(0, dtype=bool)
    if radius <= 0.0:
        raise ValueError("polygon radius must be positive")
    if sides < 3:
        raise ValueError("polygon must have at least three sides")
    rotation = math.radians(rotation_deg)
    normal_angles = rotation + (np.arange(sides, dtype=np.float32) + 0.5) * (
        2.0 * math.pi / sides
    )
    normals = np.column_stack((np.cos(normal_angles), np.sin(normal_angles)))
    apothem = radius * math.cos(math.pi / sides)
    inside_xy = np.all(xyz[:, :2] @ normals.T <= apothem + 1.0e-6, axis=1)
    inside_height = (xyz[:, 2] >= min_z) & (xyz[:, 2] <= max_z)
    return ~(inside_xy & inside_height)


def polygon_inside_mask_xy(
    xy: np.ndarray, radius: float, sides: int, rotation_deg: float = 0.0
) -> np.ndarray:
    """Return points inside the same regular polygon used for filtering."""
    if len(xy) == 0:
        return np.zeros(0, dtype=bool)
    if radius <= 0.0:
        raise ValueError("polygon radius must be positive")
    if sides < 3:
        raise ValueError("polygon must have at least three sides")
    rotation = math.radians(rotation_deg)
    normal_angles = rotation + (np.arange(sides, dtype=np.float32) + 0.5) * (
        2.0 * math.pi / sides
    )
    normals = np.column_stack((np.cos(normal_angles), np.sin(normal_angles)))
    apothem = radius * math.cos(math.pi / sides)
    return np.all(xy @ normals.T <= apothem + 1.0e-6, axis=1)


def height_ceiling_mask(xyz: np.ndarray, max_z: float) -> np.ndarray:
    """Select points at or below the navigation height ceiling."""
    return xyz[:, 2] <= max_z


def top_plate_filter_mask(
    xyz: np.ndarray,
    center_x: float,
    center_y: float,
    side_length: float,
    margin: float,
    min_z: float,
    max_z: float,
) -> np.ndarray:
    """Select points outside a horizontal regular-octagon top plate.

    The octagon is centred in ``base_link`` and has one flat side facing +X.
    ``margin`` offsets every side outwards to cover mounting tolerance and
    grazing returns without hiding low obstacles around the chassis.
    """
    apothem = side_length / (2.0 * math.tan(math.pi / 8.0)) + margin
    offset_x = xyz[:, 0] - center_x
    offset_y = xyz[:, 1] - center_y
    diagonal_limit = math.sqrt(2.0) * apothem
    inside_xy = (
        (np.abs(offset_x) <= apothem)
        & (np.abs(offset_y) <= apothem)
        & (np.abs(offset_x + offset_y) <= diagonal_limit)
        & (np.abs(offset_x - offset_y) <= diagonal_limit)
    )
    inside_height = (xyz[:, 2] >= min_z) & (xyz[:, 2] <= max_z)
    return ~(inside_xy & inside_height)


def arm_filter_mask(
    xyz: np.ndarray,
    x_min: float,
    x_max: float,
    half_width: float,
    min_z: float,
    max_z: float,
    yaw_deg: float = 0.0,
) -> np.ndarray:
    """Select points outside the rotated box occupied by the mounted arm.

    The chassis cylinder cannot cover the arm: it overhangs the front of the
    robot, so its returns sit outside ``self_filter_mask`` and would otherwise
    be marked as an obstacle the planner can never clear. This second volume
    is deliberately tight around the arm rather than a wider cylinder, so real
    obstacles beside or behind the robot are never hidden by it.
    """
    yaw = math.radians(float(yaw_deg))
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_x = cosine * xyz[:, 0] + sine * xyz[:, 1]
    local_y = -sine * xyz[:, 0] + cosine * xyz[:, 1]
    inside_forward = (local_x >= x_min) & (local_x <= x_max)
    inside_width = np.abs(local_y) <= half_width
    inside_height = (xyz[:, 2] >= min_z) & (xyz[:, 2] <= max_z)
    return ~(inside_forward & inside_width & inside_height)


def forward_arm_extent(
    xyz: np.ndarray, half_width: float, yaw_deg: float = 0.0
) -> float:
    """Largest arm-axis distance reached inside the rotated width band.

    Used to verify the configured arm box against the real cloud: if the live
    arm reaches further forward than ``x_max``, the leftover returns reappear
    as obstacles and this value says how much further the box must go.
    """
    if len(xyz) == 0:
        return 0.0
    yaw = math.radians(float(yaw_deg))
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_x = cosine * xyz[:, 0] + sine * xyz[:, 1]
    local_y = -sine * xyz[:, 0] + cosine * xyz[:, 1]
    inside_width = np.abs(local_y) <= half_width
    if not np.any(inside_width):
        return 0.0
    return float(np.max(local_x[inside_width]))


def ground_plane_removal_mask(
    xyz: np.ndarray,
    plane: np.ndarray,
    removal_below: float,
    removal_above: float,
    min_radius: float,
    max_radius: float,
) -> tuple[np.ndarray, int]:
    """Remove points in a bounded band around an accepted ground plane."""
    radial_sq = xyz[:, 0] * xyz[:, 0] + xyz[:, 1] * xyz[:, 1]
    signed_height = xyz @ plane[:3] + float(plane[3])
    ground = (
        (radial_sq >= min_radius * min_radius)
        & (radial_sq <= max_radius * max_radius)
        & (signed_height >= -removal_below)
        & (signed_height <= removal_above)
    )
    return ~ground, int(np.count_nonzero(ground))


def ground_filter_mask(
    xyz: np.ndarray,
    distance_threshold: float,
    removal_below: float,
    removal_above: float,
    max_tilt_deg: float,
    max_origin_height: float,
    min_inliers: int,
    candidate_min_z: float,
    candidate_max_z: float,
    min_radius: float,
    max_radius: float,
    iterations: int,
    removal_min_radius: float | None = None,
) -> tuple[np.ndarray, np.ndarray | None, int]:
    """Return points not belonging to a near-horizontal RANSAC ground plane.

    A plane is accepted only when it passes close to base_link z=0 and its
    normal is close to vertical.  This prevents horizontal obstacle surfaces
    from being mistaken for the floor.  The returned plane is ``[a,b,c,d]``
    for ``a*x + b*y + c*z + d = 0``.
    """
    keep_all = np.ones(len(xyz), dtype=bool)
    if len(xyz) < max(3, min_inliers):
        return keep_all, None, 0

    radial_sq = xyz[:, 0] * xyz[:, 0] + xyz[:, 1] * xyz[:, 1]
    candidates_mask = (
        (xyz[:, 2] >= candidate_min_z)
        & (xyz[:, 2] <= candidate_max_z)
        & (radial_sq >= min_radius * min_radius)
        & (radial_sq <= max_radius * max_radius)
    )
    candidates = xyz[candidates_mask]
    if len(candidates) < max(3, min_inliers):
        return keep_all, None, 0

    minimum_vertical = math.cos(math.radians(max_tilt_deg))
    generator = np.random.default_rng(0)
    best_inliers = None
    best_count = 0

    for _ in range(iterations):
        sample = candidates[generator.choice(len(candidates), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-6:
            continue
        normal = normal / norm
        if normal[2] < 0.0:
            normal = -normal
        if normal[2] < minimum_vertical:
            continue
        offset = -float(np.dot(normal, sample[0]))
        origin_height = -offset / float(normal[2])
        if abs(origin_height) > max_origin_height:
            continue
        inliers = np.abs(candidates @ normal + offset) <= distance_threshold
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < min_inliers:
        return keep_all, None, 0

    # Least-squares refinement makes the removal band stable between frames.
    floor_points = candidates[best_inliers]
    centroid = floor_points.mean(axis=0)
    _, _, axes = np.linalg.svd(floor_points - centroid, full_matrices=False)
    normal = axes[-1]
    if normal[2] < 0.0:
        normal = -normal
    if normal[2] < minimum_vertical:
        return keep_all, None, 0
    offset = -float(np.dot(normal, centroid))
    origin_height = -offset / float(normal[2])
    if abs(origin_height) > max_origin_height:
        return keep_all, None, 0

    plane = np.array([normal[0], normal[1], normal[2], offset], dtype=np.float32)
    keep, removed = ground_plane_removal_mask(
        xyz,
        plane,
        removal_below,
        removal_above,
        min_radius if removal_min_radius is None else removal_min_radius,
        max_radius,
    )
    return keep, plane, removed


@dataclass(frozen=True)
class CloudPointData:
    xyz: np.ndarray
    tag: np.ndarray | None
    point_time_ns: np.ndarray | None
    time_field: str | None


def _header_stamp_ns(message: PointCloud2) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def cloud_points_from_message(
    message: PointCloud2, max_cloud_span_s: float = 0.20
) -> CloudPointData:
    """Extract XYZ, Livox tag and absolute per-point acquisition time."""
    available = {field.name for field in message.fields}
    has_tag = "tag" in available
    time_field = next(
        (name for name in ("timestamp", "offset_time") if name in available),
        None,
    )
    field_names = ["x", "y", "z"]
    if has_tag:
        field_names.append("tag")
    if time_field is not None:
        field_names.append(time_field)
    points = pc2.read_points(
        message, field_names=tuple(field_names), skip_nans=True
    )
    if points.size == 0:
        return CloudPointData(np.empty((0, 3), dtype=np.float32), None, None, None)
    xyz = np.column_stack(
        (points["x"], points["y"], points["z"])
    ).astype(np.float32, copy=False)
    tag = np.asarray(points["tag"], dtype=np.uint8) if has_tag else None
    point_time_ns = None
    if time_field is not None:
        values = np.asarray(points[time_field])
        header_ns = _header_stamp_ns(message)
        if time_field == "offset_time":
            point_time_ns = header_ns + values.astype(np.int64)
        else:
            numeric = values.astype(np.float64)
            if np.all(np.isfinite(numeric)):
                median = float(np.median(numeric))
                if median >= 1.0e15:
                    point_time_ns = np.rint(numeric).astype(np.int64)
                elif median >= 1.0e8:
                    point_time_ns = np.rint(numeric * 1.0e9).astype(np.int64)
                else:
                    point_time_ns = header_ns + np.rint(
                        numeric * 1.0e9
                    ).astype(np.int64)
        if point_time_ns is not None:
            span_ns = int(np.max(point_time_ns)) - int(np.min(point_time_ns))
            max_span_ns = int(max_cloud_span_s * 1.0e9)
            header_gap_ns = abs(int(np.max(point_time_ns)) - header_ns)
            if (
                span_ns < 0
                or span_ns > max_span_ns
                or header_gap_ns > max_span_ns + 100_000_000
            ):
                point_time_ns = None
                time_field = None
    return CloudPointData(xyz, tag, point_time_ns, time_field)


def xyz_from_cloud(message: PointCloud2) -> np.ndarray:
    """Extract XYZ from Livox clouds that also contain mixed-type fields."""
    xyz, _ = xyz_and_tag_from_cloud(message)
    return xyz


def xyz_and_tag_from_cloud(message: PointCloud2) -> tuple[np.ndarray, np.ndarray | None]:
    """Extract XYZ plus the optional Livox confidence tag."""
    data = cloud_points_from_message(message)
    return data.xyz, data.tag


def create_cloud_with_metadata(
    header: Header,
    xyz: np.ndarray,
    tag: np.ndarray | None,
    point_time_ns: np.ndarray | None,
) -> PointCloud2:
    """Create a compact cloud while retaining tag and absolute point time."""
    xyz = np.ascontiguousarray(xyz, dtype=np.float32)
    if tag is None and point_time_ns is None:
        return pc2.create_cloud_xyz32(header, xyz)

    names = ["x", "y", "z"]
    formats = ["<f4", "<f4", "<f4"]
    offsets = [0, 4, 8]
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    point_step = 12
    if tag is not None:
        names.append("tag")
        formats.append("u1")
        offsets.append(12)
        fields.append(
            PointField(name="tag", offset=12, datatype=PointField.UINT8, count=1)
        )
        point_step = 13
    if point_time_ns is not None:
        names.append("timestamp")
        formats.append("<f8")
        offsets.append(16)
        fields.append(
            PointField(
                name="timestamp", offset=16,
                datatype=PointField.FLOAT64, count=1,
            )
        )
        point_step = 24
    dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets,
         "itemsize": point_step}
    )
    packed = np.zeros(len(xyz), dtype=dtype)
    packed["x"], packed["y"], packed["z"] = xyz.T
    if tag is not None:
        packed["tag"] = np.asarray(tag, dtype=np.uint8)
    if point_time_ns is not None:
        packed["timestamp"] = np.asarray(point_time_ns, dtype=np.float64) / 1.0e9

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(xyz)
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = point_step
    cloud.row_step = point_step * len(xyz)
    cloud.data = packed.tobytes()
    cloud.is_dense = bool(np.all(np.isfinite(xyz)))
    return cloud


class LidarSelfFilter(Node):
    def __init__(self) -> None:
        super().__init__("lidar_self_filter")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar_filtered")
        self.declare_parameter("status_topic", "/medical_nav/lidar_filter_status")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("output_min_z", -10.0)
        self.declare_parameter("output_max_z", 0.55)
        self.declare_parameter("self_radius", 0.265)
        self.declare_parameter("self_filter_shape", "cylinder")
        self.declare_parameter("self_polygon_sides", 16)
        self.declare_parameter("self_polygon_rotation_deg", 0.0)
        self.declare_parameter(
            "self_polygon_topic", "/medical_nav/self_filter_polygon"
        )
        self.declare_parameter("calibration_stop_radius", 0.285)
        self.declare_parameter("self_min_z", -0.05)
        self.declare_parameter("self_max_z", 0.65)
        self.declare_parameter("top_plate_filter_enabled", True)
        self.declare_parameter("top_plate_center_x", 0.0)
        self.declare_parameter("top_plate_center_y", 0.0)
        self.declare_parameter("top_plate_side_length", 0.18)
        self.declare_parameter("top_plate_margin", 0.03)
        self.declare_parameter("top_plate_min_z", 0.52)
        self.declare_parameter("top_plate_max_z", 0.65)
        # The mounted arm overhangs the chassis, so a base_link cylinder cannot
        # cover it. This box is the arm's own volume; it must stay tight or it
        # will hide genuine obstacles in front of the robot.
        self.declare_parameter("arm_filter_enabled", True)
        self.declare_parameter("arm_x_min", 0.10)
        self.declare_parameter("arm_x_max", 0.40)
        self.declare_parameter("arm_half_width", 0.14)
        self.declare_parameter("arm_yaw_deg", 0.0)
        self.declare_parameter("arm_min_z", 0.45)
        self.declare_parameter("arm_max_z", 0.82)
        self.declare_parameter("ground_filter_enabled", True)
        self.declare_parameter("ground_distance_threshold", 0.04)
        self.declare_parameter("ground_removal_below", 0.06)
        self.declare_parameter("ground_removal_above", 0.12)
        self.declare_parameter("ground_max_tilt_deg", 12.0)
        self.declare_parameter("ground_max_origin_height", 0.15)
        self.declare_parameter("ground_min_inliers", 300)
        self.declare_parameter("ground_candidate_min_z", -0.25)
        self.declare_parameter("ground_candidate_max_z", 0.30)
        self.declare_parameter("ground_removal_min_radius", 0.27)
        self.declare_parameter("ground_min_radius", 0.30)
        self.declare_parameter("ground_max_radius", 6.5)
        self.declare_parameter("ground_ransac_iterations", 40)
        self.declare_parameter("ground_plane_hold_s", 1.0)
        self.declare_parameter("reject_livox_noise", True)
        # Livox tag low nibble contains spatial/intensity noise confidence;
        # upper bits describe return number and must remain accepted.
        self.declare_parameter("livox_noise_mask", 15)
        self.declare_parameter("gate_on_stm32_ready", True)
        self.declare_parameter("motion_compensation_enabled", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("motion_history_s", 2.0)
        self.declare_parameter("max_motion_extrapolation_s", 0.06)
        self.declare_parameter("max_cloud_span_s", 0.20)

        get = lambda name: self.get_parameter(name).value
        self.input_topic = str(get("input_topic"))
        self.output_topic = str(get("output_topic"))
        self.status_topic = str(get("status_topic"))
        self.target_frame = str(get("target_frame"))
        self.output_min_z = float(get("output_min_z"))
        self.output_max_z = float(get("output_max_z"))
        self.self_radius = float(get("self_radius"))
        self.self_filter_shape = str(get("self_filter_shape")).strip().lower()
        self.self_polygon_sides = int(get("self_polygon_sides"))
        self.self_polygon_rotation_deg = float(get("self_polygon_rotation_deg"))
        self.self_polygon_topic = str(get("self_polygon_topic"))
        self.calibration_stop_radius = float(get("calibration_stop_radius"))
        self.self_min_z = float(get("self_min_z"))
        self.self_max_z = float(get("self_max_z"))
        self.top_plate_filter_enabled = bool(get("top_plate_filter_enabled"))
        self.top_plate_center_x = float(get("top_plate_center_x"))
        self.top_plate_center_y = float(get("top_plate_center_y"))
        self.top_plate_side_length = float(get("top_plate_side_length"))
        self.top_plate_margin = float(get("top_plate_margin"))
        self.top_plate_min_z = float(get("top_plate_min_z"))
        self.top_plate_max_z = float(get("top_plate_max_z"))
        self.arm_filter_enabled = bool(get("arm_filter_enabled"))
        self.arm_x_min = float(get("arm_x_min"))
        self.arm_x_max = float(get("arm_x_max"))
        self.arm_half_width = float(get("arm_half_width"))
        self.arm_yaw_deg = float(get("arm_yaw_deg"))
        self.arm_min_z = float(get("arm_min_z"))
        self.arm_max_z = float(get("arm_max_z"))
        self.ground_filter_enabled = bool(get("ground_filter_enabled"))
        self.ground_distance_threshold = float(get("ground_distance_threshold"))
        self.ground_removal_below = float(get("ground_removal_below"))
        self.ground_removal_above = float(get("ground_removal_above"))
        self.ground_max_tilt_deg = float(get("ground_max_tilt_deg"))
        self.ground_max_origin_height = float(get("ground_max_origin_height"))
        self.ground_min_inliers = int(get("ground_min_inliers"))
        self.ground_candidate_min_z = float(get("ground_candidate_min_z"))
        self.ground_candidate_max_z = float(get("ground_candidate_max_z"))
        self.ground_removal_min_radius = float(get("ground_removal_min_radius"))
        self.ground_min_radius = float(get("ground_min_radius"))
        self.ground_max_radius = float(get("ground_max_radius"))
        self.ground_ransac_iterations = int(get("ground_ransac_iterations"))
        self.ground_plane_hold_s = float(get("ground_plane_hold_s"))
        self.reject_livox_noise = bool(get("reject_livox_noise"))
        self.livox_noise_mask = int(get("livox_noise_mask"))
        self.gate_on_stm32_ready = bool(get("gate_on_stm32_ready"))
        self.motion_compensation_enabled = bool(
            get("motion_compensation_enabled")
        )
        self.odom_topic = str(get("odom_topic"))
        self.motion_history_s = float(get("motion_history_s"))
        self.max_motion_extrapolation_s = float(
            get("max_motion_extrapolation_s")
        )
        self.max_cloud_span_s = float(get("max_cloud_span_s"))

        if not math.isfinite(self.output_min_z) or not math.isfinite(self.output_max_z):
            raise ValueError("output z limits must be finite")
        if self.output_min_z >= self.output_max_z:
            raise ValueError("output_min_z must be lower than output_max_z")
        if self.self_radius <= 0.0:
            raise ValueError("self_radius must be positive")
        if self.self_filter_shape not in {"cylinder", "polygon"}:
            raise ValueError("self_filter_shape must be cylinder or polygon")
        if self.self_polygon_sides < 3:
            raise ValueError("self_polygon_sides must be at least three")
        if self.calibration_stop_radius <= self.self_radius:
            raise ValueError("calibration_stop_radius must exceed self radius")
        if self.self_min_z >= self.self_max_z:
            raise ValueError("self_min_z must be lower than self_max_z")
        if self.top_plate_side_length <= 0.0:
            raise ValueError("top_plate_side_length must be positive")
        if self.top_plate_margin < 0.0:
            raise ValueError("top_plate_margin cannot be negative")
        if self.top_plate_min_z >= self.top_plate_max_z:
            raise ValueError("top plate z range is invalid")
        if self.arm_half_width <= 0.0:
            raise ValueError("arm_half_width must be positive")
        if not math.isfinite(self.arm_yaw_deg):
            raise ValueError("arm_yaw_deg must be finite")
        if self.arm_x_min >= self.arm_x_max:
            raise ValueError("arm_x_min must be lower than arm_x_max")
        if self.arm_min_z >= self.arm_max_z:
            raise ValueError("arm_min_z must be lower than arm_max_z")
        if self.ground_distance_threshold <= 0.0:
            raise ValueError("ground_distance_threshold must be positive")
        if self.ground_removal_below < 0.0 or self.ground_removal_above < 0.0:
            raise ValueError("ground removal distances cannot be negative")
        if not 0.0 <= self.ground_max_tilt_deg < 90.0:
            raise ValueError("ground_max_tilt_deg must be in [0, 90)")
        if self.ground_candidate_min_z >= self.ground_candidate_max_z:
            raise ValueError("ground candidate z range is invalid")
        if self.ground_removal_min_radius < self.self_radius:
            raise ValueError("ground removal must start outside the self-filter")
        if self.ground_removal_min_radius > self.ground_min_radius:
            raise ValueError("ground removal radius must not exceed fit radius")
        if self.ground_min_radius >= self.ground_max_radius:
            raise ValueError("ground radius range is invalid")
        if self.ground_min_inliers < 3 or self.ground_ransac_iterations < 1:
            raise ValueError("ground RANSAC parameters are invalid")
        if self.ground_plane_hold_s < 0.0:
            raise ValueError("ground_plane_hold_s cannot be negative")
        if not 0 <= self.livox_noise_mask <= 255:
            raise ValueError("livox_noise_mask must fit in uint8")
        if self.motion_history_s <= 0.0:
            raise ValueError("motion_history_s must be positive")
        if self.max_motion_extrapolation_s < 0.0:
            raise ValueError("max_motion_extrapolation_s cannot be negative")
        if self.max_cloud_span_s <= 0.0:
            raise ValueError("max_cloud_span_s must be positive")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cached_source_frame = None
        self.rotation = None
        self.translation = None

        self.cloud_count = 0
        self.points_in = 0
        self.points_out = 0
        self.points_noise = 0
        self.points_top_plate = 0
        self.points_height = 0
        self.points_self = 0
        self.points_arm = 0
        self.points_ground = 0
        self.points_below_output = 0
        self.points_in_stop_zone = 0
        self.nearest_output_xy_m = None
        self.arm_forward_extent = 0.0
        self.ground_plane = None
        self.ground_plane_source = "none"
        self.cached_ground_plane = None
        self.cached_ground_plane_time = 0.0
        self.ground_fit_failures = 0
        self.ground_fallbacks = 0
        self.tag_available = False
        self.tf_drop_count = 0
        self.stm32_ready = not self.gate_on_stm32_ready
        self.stm32_gate_drops = 0
        self.last_processing_ms = 0.0
        self.max_processing_ms = 0.0
        self.last_output_age_s = 0.0
        self.max_output_age_s = 0.0
        self.odom_history: deque[MotionSample] = deque()
        self.motion_compensated_clouds = 0
        self.motion_compensation_fallbacks = 0
        self.point_time_field = None
        self.last_cloud_span_s = 0.0

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, low_latency_sensor_qos()
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10
        )
        self.self_polygon_publisher = self.create_publisher(
            PolygonStamped, self.self_polygon_topic, 1
        )
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cloud_callback,
            low_latency_sensor_qos(),
        )
        self.create_subscription(
            Bool,
            "/medical_nav/stm32_ready",
            self._stm32_ready,
            10,
        )
        self.create_subscription(Odometry, self.odom_topic, self._odom, 100)
        self.create_timer(2.0, self._publish_status)
        self.create_timer(1.0, self._publish_self_polygon)

        if self.arm_filter_enabled:
            arm_description = (
                f"x[{self.arm_x_min:.2f}, {self.arm_x_max:.2f}] "
                f"y+-{self.arm_half_width:.2f} "
                f"z[{self.arm_min_z:.2f}, {self.arm_max_z:.2f}] "
                f"yaw={self.arm_yaw_deg:.1f}deg"
            )
        else:
            arm_description = "off"

        self.get_logger().info(
            f"Self filter {self.input_topic} -> {self.output_topic} in "
            f"{self.target_frame}: self={self.self_filter_shape} "
            f"radius={self.self_radius:.3f} m, "
            f"z=[{self.self_min_z:.2f}, {self.self_max_z:.2f}] m, "
            f"output_z=[{self.output_min_z:.2f}, {self.output_max_z:.2f}] m, "
            f"top_plate={'on' if self.top_plate_filter_enabled else 'off'}, "
            f"arm={arm_description}, "
            f"ground={'on' if self.ground_filter_enabled else 'off'}, "
            f"stm32_gate={'on' if self.gate_on_stm32_ready else 'off'}, "
            f"deskew={'on' if self.motion_compensation_enabled else 'off'}, "
            "cloud_qos=best_effort/keep_last(1)"
        )

    @staticmethod
    def _yaw_from_odometry(message: Odometry) -> float:
        q = message.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _odom(self, message: Odometry) -> None:
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns <= 0:
            return
        sample = MotionSample(
            stamp_ns=stamp_ns,
            x=float(message.pose.pose.position.x),
            y=float(message.pose.pose.position.y),
            yaw=self._yaw_from_odometry(message),
            body_vx=float(message.twist.twist.linear.x),
            body_vy=float(message.twist.twist.linear.y),
            yaw_rate=float(message.twist.twist.angular.z),
        )
        if self.odom_history and stamp_ns < self.odom_history[-1].stamp_ns:
            self.odom_history.clear()
        if self.odom_history and stamp_ns == self.odom_history[-1].stamp_ns:
            self.odom_history[-1] = sample
        else:
            self.odom_history.append(sample)
        cutoff_ns = stamp_ns - int(self.motion_history_s * 1.0e9)
        while self.odom_history and self.odom_history[0].stamp_ns < cutoff_ns:
            self.odom_history.popleft()

    def _stm32_ready(self, message: Bool) -> None:
        ready = bool(message.data) or not self.gate_on_stm32_ready
        if ready == self.stm32_ready:
            return
        self.stm32_ready = ready
        if not ready:
            # Never reuse a floor estimate or transformed cloud state across a
            # reset followed by manual repositioning to the start point.
            self.cached_ground_plane = None
            self.cached_ground_plane_time = 0.0
            self.ground_plane = None
            self.ground_plane_source = "stm32_gate"
            self.odom_history.clear()
            self.get_logger().warn(
                "STM32 unavailable; suppressing navigation point clouds"
            )
        else:
            self.get_logger().info(
                "STM32 recovered; resuming navigation point clouds"
            )

    def _load_transform(self, source_frame: str) -> bool:
        if source_frame == self.cached_source_frame and self.rotation is not None:
            return True
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            ).transform
        except TransformException as error:
            self.tf_drop_count += 1
            if self.tf_drop_count == 1 or self.tf_drop_count % 50 == 0:
                self.get_logger().warning(
                    f"Waiting for {source_frame} -> {self.target_frame} TF: {error}"
                )
            return False

        q = transform.rotation
        t = transform.translation
        self.rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
        self.translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        self.cached_source_frame = source_frame
        return True

    def _cloud_callback(self, message: PointCloud2) -> None:
        callback_started_ns = time.perf_counter_ns()
        if not self.stm32_ready:
            self.stm32_gate_drops += 1
            return
        source_frame = message.header.frame_id.lstrip("/")
        if not source_frame or not self._load_transform(source_frame):
            return

        cloud_data = cloud_points_from_message(message, self.max_cloud_span_s)
        xyz = cloud_data.xyz
        tag = cloud_data.tag
        if xyz.size == 0:
            return

        points_in = len(xyz)
        selected = np.arange(points_in)
        noise_count = 0
        self.tag_available = tag is not None
        self.point_time_field = cloud_data.time_field
        if self.reject_livox_noise and tag is not None:
            keep_confident = (tag & self.livox_noise_mask) == 0
            noise_count = points_in - int(np.count_nonzero(keep_confident))
            xyz = xyz[keep_confident]
            selected = selected[keep_confident]

        xyz = transform_points(xyz, self.rotation, self.translation)
        output_stamp_ns = _header_stamp_ns(message)
        source_point_time_ns = (
            cloud_data.point_time_ns[selected]
            if cloud_data.point_time_ns is not None
            else None
        )
        reference_ns = None
        self.last_cloud_span_s = 0.0
        if source_point_time_ns is not None and len(source_point_time_ns):
            reference_ns = int(np.max(source_point_time_ns))
            self.last_cloud_span_s = (
                reference_ns - int(np.min(source_point_time_ns))
            ) / 1.0e9

        # Chassis, top-plate and arm returns move with base_link rather than
        # the static world. Remove them in their acquisition-time base frame
        # before deskewing; otherwise forward motion shifts rear self returns
        # outside the self-filter and creates a speed-dependent ghost trail.
        top_plate_removed = 0
        if self.top_plate_filter_enabled:
            keep_top_plate = top_plate_filter_mask(
                xyz,
                self.top_plate_center_x,
                self.top_plate_center_y,
                self.top_plate_side_length,
                self.top_plate_margin,
                self.top_plate_min_z,
                self.top_plate_max_z,
            )
            top_plate_removed = len(xyz) - int(np.count_nonzero(keep_top_plate))
            xyz = xyz[keep_top_plate]
            selected = selected[keep_top_plate]
        keep_height = height_ceiling_mask(xyz, self.output_max_z)
        height_removed = len(xyz) - int(np.count_nonzero(keep_height))
        xyz = xyz[keep_height]
        selected = selected[keep_height]
        if self.self_filter_shape == "polygon":
            keep_self = polygon_self_filter_mask(
                xyz, self.self_radius, self.self_polygon_sides,
                self.self_polygon_rotation_deg, self.self_min_z, self.self_max_z
            )
        else:
            keep_self = self_filter_mask(
                xyz, self.self_radius, self.self_min_z, self.self_max_z
            )
        self_removed = len(xyz) - int(np.count_nonzero(keep_self))
        xyz = xyz[keep_self]
        selected = selected[keep_self]
        arm_removed = 0
        if self.arm_filter_enabled:
            # Measured before removal, so the operator can see whether the live
            # arm still reaches past the configured box.
            self.arm_forward_extent = forward_arm_extent(
                xyz, self.arm_half_width, self.arm_yaw_deg
            )
            keep_arm = arm_filter_mask(
                xyz,
                self.arm_x_min,
                self.arm_x_max,
                self.arm_half_width,
                self.arm_min_z,
                self.arm_max_z,
                self.arm_yaw_deg,
            )
            arm_removed = len(xyz) - int(np.count_nonzero(keep_arm))
            xyz = xyz[keep_arm]
            selected = selected[keep_arm]
        else:
            self.arm_forward_extent = 0.0

        point_time_ns = (
            cloud_data.point_time_ns[selected]
            if cloud_data.point_time_ns is not None
            else None
        )
        if (
            reference_ns is not None
            and point_time_ns is not None
            and len(point_time_ns)
            and self.motion_compensation_enabled
            and self.target_frame == "base_link"
        ):
            compensated = deskew_points(
                xyz,
                point_time_ns,
                tuple(self.odom_history),
                reference_ns,
                int(self.max_motion_extrapolation_s * 1.0e9),
            )
            if compensated is not None:
                xyz = compensated
                output_stamp_ns = reference_ns
                self.motion_compensated_clouds += 1
            else:
                self.motion_compensation_fallbacks += 1
        elif self.motion_compensation_enabled and reference_ns is None:
            self.motion_compensation_fallbacks += 1

        plane = None
        ground_count = 0
        ground_plane_source = "off"
        if self.ground_filter_enabled:
            keep_ground, plane, ground_count = ground_filter_mask(
                xyz,
                self.ground_distance_threshold,
                self.ground_removal_below,
                self.ground_removal_above,
                self.ground_max_tilt_deg,
                self.ground_max_origin_height,
                self.ground_min_inliers,
                self.ground_candidate_min_z,
                self.ground_candidate_max_z,
                self.ground_min_radius,
                self.ground_max_radius,
                self.ground_ransac_iterations,
                self.ground_removal_min_radius,
            )
            now = time.monotonic()
            if plane is not None:
                self.cached_ground_plane = plane.copy()
                self.cached_ground_plane_time = now
                ground_plane_source = "live"
            else:
                self.ground_fit_failures += 1
                cache_age = now - self.cached_ground_plane_time
                if (
                    self.cached_ground_plane is not None
                    and cache_age <= self.ground_plane_hold_s
                ):
                    keep_ground, ground_count = ground_plane_removal_mask(
                        xyz,
                        self.cached_ground_plane,
                        self.ground_removal_below,
                        self.ground_removal_above,
                        self.ground_removal_min_radius,
                        self.ground_max_radius,
                    )
                    plane = self.cached_ground_plane
                    ground_plane_source = "cached"
                    self.ground_fallbacks += 1
                else:
                    ground_plane_source = "none"
            xyz = xyz[keep_ground]
            selected = selected[keep_ground]
        keep_output_height = xyz[:, 2] >= self.output_min_z
        below_output_count = len(xyz) - int(
            np.count_nonzero(keep_output_height)
        )
        filtered = np.ascontiguousarray(
            xyz[keep_output_height], dtype=np.float32
        )
        selected = selected[keep_output_height]

        header = Header()
        header.stamp.sec = int(output_stamp_ns // 1_000_000_000)
        header.stamp.nanosec = int(output_stamp_ns % 1_000_000_000)
        header.frame_id = self.target_frame
        filtered_tag = tag[selected] if tag is not None else None
        filtered_time_ns = (
            cloud_data.point_time_ns[selected]
            if cloud_data.point_time_ns is not None
            else None
        )
        self.publisher.publish(
            create_cloud_with_metadata(
                header, filtered, filtered_tag, filtered_time_ns
            )
        )

        output_age_s = max(
            0.0, (self.get_clock().now().nanoseconds - output_stamp_ns) / 1.0e9
        )
        processing_ms = (time.perf_counter_ns() - callback_started_ns) / 1.0e6

        self.cloud_count += 1
        self.points_in = points_in
        self.points_out = len(filtered)
        self.points_noise = noise_count
        self.points_top_plate = top_plate_removed
        self.points_height = height_removed
        self.points_self = self_removed
        self.points_arm = arm_removed
        self.points_ground = ground_count
        self.points_below_output = below_output_count
        if len(filtered):
            output_ranges = np.linalg.norm(filtered[:, :2], axis=1)
            self.nearest_output_xy_m = float(np.min(output_ranges))
            self.points_in_stop_zone = int(
                np.count_nonzero(
                    polygon_inside_mask_xy(
                        filtered[:, :2],
                        self.calibration_stop_radius,
                        self.self_polygon_sides,
                        self.self_polygon_rotation_deg,
                    )
                )
            )
        else:
            self.nearest_output_xy_m = None
            self.points_in_stop_zone = 0
        self.ground_plane = plane
        self.ground_plane_source = ground_plane_source
        self.last_processing_ms = processing_ms
        self.max_processing_ms = max(self.max_processing_ms, processing_ms)
        self.last_output_age_s = output_age_s
        self.max_output_age_s = max(self.max_output_age_s, output_age_s)

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "clouds": self.cloud_count,
                "points_in": self.points_in,
                "points_out": self.points_out,
                "points_noise": self.points_noise,
                "points_top_plate": self.points_top_plate,
                "points_height": self.points_height,
                "points_self": self.points_self,
                "points_arm": self.points_arm,
                "points_ground": self.points_ground,
                "points_below_output": self.points_below_output,
                "points_in_stop_zone": self.points_in_stop_zone,
                "nearest_output_xy_m": (
                    round(self.nearest_output_xy_m, 3)
                    if self.nearest_output_xy_m is not None else None
                ),
                "ground_plane_source": self.ground_plane_source,
                "ground_fit_failures": self.ground_fit_failures,
                "ground_fallbacks": self.ground_fallbacks,
                "tf_drops": self.tf_drop_count,
                "stm32_ready": self.stm32_ready,
                "stm32_gate_enabled": self.gate_on_stm32_ready,
                "stm32_gate_drops": self.stm32_gate_drops,
                "output_max_z_m": self.output_max_z,
                "output_min_z_m": self.output_min_z,
                "processing_ms": round(self.last_processing_ms, 3),
                "max_processing_ms": round(self.max_processing_ms, 3),
                "output_age_s": round(self.last_output_age_s, 4),
                "max_output_age_s": round(self.max_output_age_s, 4),
                "self_radius_m": self.self_radius,
                "self_filter_shape": self.self_filter_shape,
                "self_polygon_sides": self.self_polygon_sides,
                "self_polygon_rotation_deg": self.self_polygon_rotation_deg,
                "self_polygon_topic": self.self_polygon_topic,
                "calibration_stop_radius_m": self.calibration_stop_radius,
                "stop_detection_band_m": round(
                    self.calibration_stop_radius - self.self_radius, 3
                ),
                "top_plate_filter": self.top_plate_filter_enabled,
                "top_plate": (
                    [
                        self.top_plate_center_x,
                        self.top_plate_center_y,
                        self.top_plate_side_length,
                        self.top_plate_margin,
                        self.top_plate_min_z,
                        self.top_plate_max_z,
                    ]
                    if self.top_plate_filter_enabled
                    else None
                ),
                "arm_filter": self.arm_filter_enabled,
                "arm_box": (
                    [
                        self.arm_x_min,
                        self.arm_x_max,
                        self.arm_half_width,
                        self.arm_min_z,
                        self.arm_max_z,
                        self.arm_yaw_deg,
                    ]
                    if self.arm_filter_enabled
                    else None
                ),
                # Forward reach of the returns inside the arm's width band. If
                # this exceeds arm_x_max the box is too short and the leftover
                # returns will show up as obstacles again.
                "arm_forward_extent_m": round(self.arm_forward_extent, 3),
                "ground_filter": self.ground_filter_enabled,
                "livox_noise_filter": self.reject_livox_noise,
                "tag_available": self.tag_available,
                "point_time_field": self.point_time_field,
                "cloud_span_s": round(self.last_cloud_span_s, 4),
                "motion_compensation_enabled": self.motion_compensation_enabled,
                "motion_compensated_clouds": self.motion_compensated_clouds,
                "motion_compensation_fallbacks": (
                    self.motion_compensation_fallbacks
                ),
                "odom_history_samples": len(self.odom_history),
                "ground_plane": (
                    [round(float(value), 5) for value in self.ground_plane]
                    if self.ground_plane is not None
                    else None
                ),
            }
        )
        self.status_publisher.publish(message)

    def _publish_self_polygon(self) -> None:
        """Publish the exact self-filter boundary for RViz calibration."""
        message = PolygonStamped()
        message.header.frame_id = self.target_frame
        message.header.stamp = self.get_clock().now().to_msg()
        if self.self_filter_shape == "polygon":
            vertices = regular_polygon_vertices(
                self.self_radius, self.self_polygon_sides,
                self.self_polygon_rotation_deg
            )
        else:
            vertices = regular_polygon_vertices(self.self_radius, 64, 0.0)
        message.polygon.points = [
            Point32(x=float(x), y=float(y), z=0.0) for x, y in vertices
        ]
        self.self_polygon_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSelfFilter()
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
