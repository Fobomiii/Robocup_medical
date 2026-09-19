#!/usr/bin/env python3
"""Transform Mid360 clouds and remove chassis plus floor returns."""

import json
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener


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
    signed_height = xyz @ normal + offset
    ground = (
        (radial_sq >= min_radius * min_radius)
        & (radial_sq <= max_radius * max_radius)
        & (signed_height >= -removal_below)
        & (signed_height <= removal_above)
    )
    return ~ground, plane, int(np.count_nonzero(ground))


def xyz_from_cloud(message: PointCloud2) -> np.ndarray:
    """Extract XYZ from Livox clouds that also contain mixed-type fields."""
    xyz, _ = xyz_and_tag_from_cloud(message)
    return xyz


def xyz_and_tag_from_cloud(message: PointCloud2) -> tuple[np.ndarray, np.ndarray | None]:
    """Extract XYZ plus the optional Livox confidence tag."""
    has_tag = any(field.name == "tag" for field in message.fields)
    field_names = ("x", "y", "z", "tag") if has_tag else ("x", "y", "z")
    points = pc2.read_points(
        message, field_names=field_names, skip_nans=True
    )
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32), None
    xyz = np.column_stack(
        (points["x"], points["y"], points["z"])
    ).astype(np.float32, copy=False)
    tag = np.asarray(points["tag"], dtype=np.uint8) if has_tag else None
    return xyz, tag


class LidarSelfFilter(Node):
    def __init__(self) -> None:
        super().__init__("lidar_self_filter")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar_filtered")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("self_radius", 0.26)
        self.declare_parameter("self_min_z", -0.05)
        self.declare_parameter("self_max_z", 0.65)
        self.declare_parameter("ground_filter_enabled", True)
        self.declare_parameter("ground_distance_threshold", 0.04)
        self.declare_parameter("ground_removal_below", 0.05)
        self.declare_parameter("ground_removal_above", 0.08)
        self.declare_parameter("ground_max_tilt_deg", 12.0)
        self.declare_parameter("ground_max_origin_height", 0.15)
        self.declare_parameter("ground_min_inliers", 300)
        self.declare_parameter("ground_candidate_min_z", -0.25)
        self.declare_parameter("ground_candidate_max_z", 0.30)
        self.declare_parameter("ground_min_radius", 0.30)
        self.declare_parameter("ground_max_radius", 5.0)
        self.declare_parameter("ground_ransac_iterations", 40)
        self.declare_parameter("reject_livox_noise", True)
        # Livox tag low nibble contains spatial/intensity noise confidence;
        # upper bits describe return number and must remain accepted.
        self.declare_parameter("livox_noise_mask", 15)

        get = lambda name: self.get_parameter(name).value
        self.input_topic = str(get("input_topic"))
        self.output_topic = str(get("output_topic"))
        self.target_frame = str(get("target_frame"))
        self.self_radius = float(get("self_radius"))
        self.self_min_z = float(get("self_min_z"))
        self.self_max_z = float(get("self_max_z"))
        self.ground_filter_enabled = bool(get("ground_filter_enabled"))
        self.ground_distance_threshold = float(get("ground_distance_threshold"))
        self.ground_removal_below = float(get("ground_removal_below"))
        self.ground_removal_above = float(get("ground_removal_above"))
        self.ground_max_tilt_deg = float(get("ground_max_tilt_deg"))
        self.ground_max_origin_height = float(get("ground_max_origin_height"))
        self.ground_min_inliers = int(get("ground_min_inliers"))
        self.ground_candidate_min_z = float(get("ground_candidate_min_z"))
        self.ground_candidate_max_z = float(get("ground_candidate_max_z"))
        self.ground_min_radius = float(get("ground_min_radius"))
        self.ground_max_radius = float(get("ground_max_radius"))
        self.ground_ransac_iterations = int(get("ground_ransac_iterations"))
        self.reject_livox_noise = bool(get("reject_livox_noise"))
        self.livox_noise_mask = int(get("livox_noise_mask"))

        if self.self_radius <= 0.0:
            raise ValueError("self_radius must be positive")
        if self.self_min_z >= self.self_max_z:
            raise ValueError("self_min_z must be lower than self_max_z")
        if self.ground_distance_threshold <= 0.0:
            raise ValueError("ground_distance_threshold must be positive")
        if self.ground_removal_below < 0.0 or self.ground_removal_above < 0.0:
            raise ValueError("ground removal distances cannot be negative")
        if not 0.0 <= self.ground_max_tilt_deg < 90.0:
            raise ValueError("ground_max_tilt_deg must be in [0, 90)")
        if self.ground_candidate_min_z >= self.ground_candidate_max_z:
            raise ValueError("ground candidate z range is invalid")
        if self.ground_min_radius < self.self_radius:
            raise ValueError("ground_min_radius must cover the self-filter radius")
        if self.ground_min_radius >= self.ground_max_radius:
            raise ValueError("ground radius range is invalid")
        if self.ground_min_inliers < 3 or self.ground_ransac_iterations < 1:
            raise ValueError("ground RANSAC parameters are invalid")
        if not 0 <= self.livox_noise_mask <= 255:
            raise ValueError("livox_noise_mask must fit in uint8")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cached_source_frame = None
        self.rotation = None
        self.translation = None

        self.cloud_count = 0
        self.points_in = 0
        self.points_out = 0
        self.points_noise = 0
        self.points_self = 0
        self.points_ground = 0
        self.ground_plane = None
        self.tag_available = False
        self.tf_drop_count = 0

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.status_publisher = self.create_publisher(
            String, "/medical_nav/lidar_filter_status", 10
        )
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f"Self filter {self.input_topic} -> {self.output_topic} in "
            f"{self.target_frame}: radius={self.self_radius:.3f} m, "
            f"z=[{self.self_min_z:.2f}, {self.self_max_z:.2f}] m, "
            f"ground={'on' if self.ground_filter_enabled else 'off'}"
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
        source_frame = message.header.frame_id.lstrip("/")
        if not source_frame or not self._load_transform(source_frame):
            return

        xyz, tag = xyz_and_tag_from_cloud(message)
        if xyz.size == 0:
            return

        points_in = len(xyz)
        noise_count = 0
        self.tag_available = tag is not None
        if self.reject_livox_noise and tag is not None:
            keep_confident = (tag & self.livox_noise_mask) == 0
            noise_count = points_in - int(np.count_nonzero(keep_confident))
            xyz = xyz[keep_confident]

        xyz = transform_points(xyz, self.rotation, self.translation)
        keep_self = self_filter_mask(
            xyz, self.self_radius, self.self_min_z, self.self_max_z
        )
        without_self = xyz[keep_self]
        plane = None
        ground_count = 0
        if self.ground_filter_enabled:
            keep_ground, plane, ground_count = ground_filter_mask(
                without_self,
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
            )
            without_self = without_self[keep_ground]
        filtered = np.ascontiguousarray(without_self, dtype=np.float32)

        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = self.target_frame
        self.publisher.publish(pc2.create_cloud_xyz32(header, filtered))

        self.cloud_count += 1
        self.points_in = points_in
        self.points_out = len(filtered)
        self.points_noise = noise_count
        self.points_self = len(xyz) - int(np.count_nonzero(keep_self))
        self.points_ground = ground_count
        self.ground_plane = plane

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "clouds": self.cloud_count,
                "points_in": self.points_in,
                "points_out": self.points_out,
                "points_noise": self.points_noise,
                "points_self": self.points_self,
                "points_ground": self.points_ground,
                "tf_drops": self.tf_drop_count,
                "self_radius_m": self.self_radius,
                "ground_filter": self.ground_filter_enabled,
                "livox_noise_filter": self.reject_livox_noise,
                "tag_available": self.tag_available,
                "ground_plane": (
                    [round(float(value), 5) for value in self.ground_plane]
                    if self.ground_plane is not None
                    else None
                ),
            }
        )
        self.status_publisher.publish(message)


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
