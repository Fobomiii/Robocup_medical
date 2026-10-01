#!/usr/bin/env python3
"""Expand compact cone returns into their known ground footprint."""

import json
import math
import time

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from obstacle_detector.cone_footprint_core import expand_cone_footprints, footprint_disk
from obstacle_detector.lidar_transform import quaternion_matrix, transform_points, xyz_from_cloud


class ConeFootprintCompensator(Node):
    def __init__(self) -> None:
        super().__init__("cone_footprint_compensator")
        self.declare_parameter("input_topic", "/livox/lidar_filtered")
        self.declare_parameter("output_topic", "/livox/lidar_nav")
        self.declare_parameter("base_radius", 0.18)
        self.declare_parameter("physical_base_radius", 0.155)
        self.declare_parameter("cone_height", 0.65)
        self.declare_parameter("min_z", 0.08)
        self.declare_parameter("min_range", 0.30)
        self.declare_parameter("max_range", 4.5)
        self.declare_parameter("cluster_cell", 0.08)
        self.declare_parameter("min_points", 3)
        self.declare_parameter("max_span", 0.32)
        self.declare_parameter("min_vertical_span", 0.06)
        self.declare_parameter("disk_spacing", 0.04)
        self.declare_parameter("disk_height", 0.12)
        # A cone always has returns at ground level.  A compact cluster whose
        # lowest return is above this is not a cone -- the mounted arm at
        # 0.64-0.70 m is the case that matters, because stamping a disk for it
        # would put a false wall at z=0.12 m right in front of the robot.
        self.declare_parameter("min_base_z", 0.35)
        # Keep only the synthetic cone-base disk briefly.  The full cloud is
        # still cleared every frame so moving people do not leave a trail.
        self.declare_parameter("persistence_s", 0.60)
        self.declare_parameter("persistence_frame", "odom")
        self.declare_parameter("persistence_match_distance", 0.40)

        get = lambda name: self.get_parameter(name).value
        self.input_topic = str(get("input_topic"))
        self.output_topic = str(get("output_topic"))
        self.parameters = {
            "base_radius": float(get("base_radius")),
            "physical_base_radius": float(get("physical_base_radius")),
            "cone_height": float(get("cone_height")),
            "min_z": float(get("min_z")),
            "min_range": float(get("min_range")),
            "max_range": float(get("max_range")),
            "cluster_cell": float(get("cluster_cell")),
            "min_points": int(get("min_points")),
            "max_span": float(get("max_span")),
            "min_vertical_span": float(get("min_vertical_span")),
            "disk_spacing": float(get("disk_spacing")),
            "disk_height": float(get("disk_height")),
            "min_base_z": float(get("min_base_z")),
        }
        self.persistence_s = float(get("persistence_s"))
        self.persistence_frame = str(get("persistence_frame")).lstrip("/")
        self.persistence_match_distance = float(get("persistence_match_distance"))
        if self.parameters["base_radius"] <= 0.0:
            raise ValueError("base_radius must be positive")
        if self.parameters["physical_base_radius"] <= 0.0:
            raise ValueError("physical_base_radius must be positive")
        if self.parameters["physical_base_radius"] > self.parameters["base_radius"]:
            raise ValueError("physical_base_radius must not exceed base_radius")
        if self.parameters["cone_height"] <= self.parameters["min_z"]:
            raise ValueError("cone_height must exceed min_z")
        if self.parameters["cluster_cell"] <= 0.0:
            raise ValueError("cluster_cell must be positive")
        if self.parameters["min_base_z"] < self.parameters["min_z"]:
            raise ValueError("min_base_z must not be below min_z")
        if self.parameters["disk_spacing"] <= 0.0:
            raise ValueError("disk_spacing must be positive")
        if self.persistence_s < 0.0:
            raise ValueError("persistence_s cannot be negative")
        if not self.persistence_frame:
            raise ValueError("persistence_frame must not be empty")
        if self.persistence_match_distance <= 0.0:
            raise ValueError("persistence_match_distance must be positive")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._held_centers: list[dict[str, object]] = []
        self._last_tf_warning = 0.0
        self.cloud_count = 0
        self.detected_count = 0
        self.held_count = 0
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.status_publisher = self.create_publisher(
            String, "/medical_nav/cone_compensation_status", 10
        )
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(2.0, self._publish_status)
        self.get_logger().info(
            f"Cone footprint {self.input_topic} -> {self.output_topic}: "
            f"height={self.parameters['cone_height']:.3f} m, "
            f"base_radius={self.parameters['base_radius']:.3f} m, "
            f"persistence={self.persistence_s:.2f} s"
        )

    @staticmethod
    def _transform_for(transform) -> tuple[np.ndarray, np.ndarray]:
        q = transform.rotation
        t = transform.translation
        return (
            quaternion_matrix(q.x, q.y, q.z, q.w),
            np.array([t.x, t.y, t.z], dtype=np.float32),
        )

    def _lookup(self, target_frame: str, source_frame: str, stamp=None):
        if target_frame == source_frame:
            return None
        try:
            # Use the cloud timestamp for the source-to-odom conversion.  For
            # the reverse conversion, the caller passes a zero stamp so the
            # latest robot pose is used and short-lived tracks follow motion.
            lookup_time = (
                Time.from_msg(stamp)
                if stamp is not None and (stamp.sec or stamp.nanosec)
                else Time()
            )
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                lookup_time,
                timeout=Duration(seconds=0.05),
            ).transform
        except TransformException as error:
            now = time.monotonic()
            if now - self._last_tf_warning > 2.0:
                self.get_logger().warning(
                    f"Cone compensation TF {source_frame}->{target_frame} unavailable: {error}"
                )
                self._last_tf_warning = now
            return False

    def _to_persistence_frame(
        self, centers: list[tuple[float, float]], source_frame: str, stamp
    ) -> np.ndarray | None:
        if not centers:
            return np.empty((0, 2), dtype=np.float32)
        transform = self._lookup(self.persistence_frame, source_frame, stamp)
        if transform is False:
            return None
        if transform is None:
            return np.asarray(centers, dtype=np.float32)
        rotation, translation = self._transform_for(transform)
        points = np.column_stack(
            (np.asarray(centers, dtype=np.float32), np.zeros(len(centers), dtype=np.float32))
        )
        return transform_points(points, rotation, translation)[:, :2]

    def _from_persistence_frame(
        self, centers: np.ndarray, target_frame: str
    ) -> np.ndarray | None:
        if len(centers) == 0:
            return np.empty((0, 2), dtype=np.float32)
        transform = self._lookup(target_frame, self.persistence_frame)
        if transform is False:
            return None
        if transform is None:
            return centers.astype(np.float32, copy=False)
        rotation, translation = self._transform_for(transform)
        points = np.column_stack(
            (centers, np.zeros(len(centers), dtype=np.float32))
        )
        return transform_points(points, rotation, translation)[:, :2]

    def _persistent_extras(
        self,
        centers: list[tuple[float, float]],
        source_frame: str,
        stamp,
    ) -> list[tuple[float, float]]:
        now = time.monotonic()
        self._held_centers = [
            item for item in self._held_centers
            if now - float(item["seen"]) <= self.persistence_s
        ]
        if self.persistence_s <= 0.0:
            self.held_count = 0
            return []

        current_fixed = self._to_persistence_frame(centers, source_frame, stamp)
        if current_fixed is None:
            self.held_count = 0
            return []

        # Match detections in a fixed frame so a moving robot does not turn a
        # previous base_link coordinate into a false obstacle trail.
        matched_tracks: set[int] = set()
        for point in current_fixed:
            match = None
            if self._held_centers:
                candidates = [
                    (index, math.hypot(
                        float(point[0] - item["center"][0]),
                        float(point[1] - item["center"][1]),
                    ))
                    for index, item in enumerate(self._held_centers)
                    if index not in matched_tracks
                ]
                if candidates:
                    nearest, distance = min(candidates, key=lambda pair: pair[1])
                    if distance <= self.persistence_match_distance:
                        matched_tracks.add(nearest)
                        match = self._held_centers[nearest]
            if match is None:
                self._held_centers.append(
                    {"center": point.astype(np.float32), "seen": now}
                )
            else:
                match["center"] = point.astype(np.float32)
                match["seen"] = now

        if not self._held_centers:
            self.held_count = 0
            return []
        active_fixed = np.asarray(
            [item["center"] for item in self._held_centers], dtype=np.float32
        )
        active_base = self._from_persistence_frame(active_fixed, source_frame)
        if active_base is None:
            self.held_count = 0
            return []

        extras = []
        for point in active_base:
            if centers:
                nearest = min(
                    math.hypot(float(point[0] - x), float(point[1] - y))
                    for x, y in centers
                )
                if nearest < self.parameters["disk_spacing"] * 0.75:
                    continue
            extras.append((float(point[0]), float(point[1])))
        self.held_count = len(extras)
        return extras

    def _cloud_callback(self, message: PointCloud2) -> None:
        xyz = xyz_from_cloud(message)
        source_frame = message.header.frame_id.lstrip("/") or "base_link"
        expanded, centers = expand_cone_footprints(xyz, **self.parameters)
        extras = self._persistent_extras(centers, source_frame, message.header.stamp)
        if extras:
            expanded = np.vstack(
                [
                    expanded,
                    *[
                        footprint_disk(
                            x,
                            y,
                            self.parameters["base_radius"],
                            self.parameters["disk_spacing"],
                            self.parameters["disk_height"],
                        )
                        for x, y in extras
                    ],
                ]
            ).astype(np.float32, copy=False)
        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = message.header.frame_id
        self.publisher.publish(pc2.create_cloud_xyz32(header, expanded))
        self.cloud_count += 1
        self.detected_count = len(centers)

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "clouds": self.cloud_count,
                "cones": self.detected_count,
                "held_cones": self.held_count,
                "base_radius_m": self.parameters["base_radius"],
                "physical_base_radius_m": self.parameters["physical_base_radius"],
                "cone_height_m": self.parameters["cone_height"],
                "persistence_s": self.persistence_s,
            }
        )
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ConeFootprintCompensator()
    # Cone persistence uses exact-time base_link<->odom lookups. Keep cloud
    # callbacks serialized, but allow TransformListener (which uses its own
    # reentrant callback group) to update the buffer on the second worker
    # thread instead of starving behind point-cloud processing.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
