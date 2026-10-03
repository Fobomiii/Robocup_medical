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
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from obstacle_detector.cone_footprint_core import (
    expand_cone_footprints,
    footprint_disk,
    future_timestamp_gap_s,
)
from obstacle_detector.lidar_transform import quaternion_matrix, transform_points, xyz_from_cloud
from obstacle_detector.qos_profiles import low_latency_sensor_qos


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
        self.declare_parameter("min_points", 5)
        self.declare_parameter("max_span", 0.32)
        self.declare_parameter("min_vertical_span", 0.12)
        self.declare_parameter("center_merge_distance", 0.28)
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
        # Exact cloud-time TF remains preferred. If the cloud is only slightly
        # newer than the latest odom TF, use that latest TF instead of dropping
        # persistence for the frame. Never bridge a genuinely stale TF gap.
        self.declare_parameter("tf_future_fallback_enabled", True)
        self.declare_parameter("tf_future_fallback_max_gap_s", 0.20)

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
            "center_merge_distance": float(get("center_merge_distance")),
            "disk_spacing": float(get("disk_spacing")),
            "disk_height": float(get("disk_height")),
            "min_base_z": float(get("min_base_z")),
        }
        self.persistence_s = float(get("persistence_s"))
        self.persistence_frame = str(get("persistence_frame")).lstrip("/")
        self.persistence_match_distance = float(get("persistence_match_distance"))
        self.tf_future_fallback_enabled = bool(get("tf_future_fallback_enabled"))
        self.tf_future_fallback_max_gap_s = float(
            get("tf_future_fallback_max_gap_s")
        )
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
        if self.parameters["min_points"] < 1:
            raise ValueError("min_points must be positive")
        if self.parameters["min_vertical_span"] <= 0.0:
            raise ValueError("min_vertical_span must be positive")
        if self.parameters["center_merge_distance"] <= 0.0:
            raise ValueError("center_merge_distance must be positive")
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
        if self.tf_future_fallback_max_gap_s <= 0.0:
            raise ValueError("tf_future_fallback_max_gap_s must be positive")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._held_centers: list[dict[str, object]] = []
        self._last_tf_warning = 0.0
        self._last_tf_fallback_log = 0.0
        self.tf_future_fallback_count = 0
        self.tf_lookup_failure_count = 0
        self.tf_last_fallback_gap_s: float | None = None
        self.tf_last_rejected_gap_s: float | None = None
        self.cloud_count = 0
        self.raw_cluster_count = 0
        self.detected_count = 0
        self.held_count = 0
        self._current_centers_base: list[tuple[float, float]] = []
        self._held_centers_base: list[tuple[float, float]] = []
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, low_latency_sensor_qos()
        )
        self.status_publisher = self.create_publisher(
            String, "/medical_nav/cone_compensation_status", 10
        )
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cloud_callback,
            low_latency_sensor_qos(),
        )
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            f"Cone footprint {self.input_topic} -> {self.output_topic}: "
            f"height={self.parameters['cone_height']:.3f} m, "
            f"base_radius={self.parameters['base_radius']:.3f} m, "
            f"merge_distance={self.parameters['center_merge_distance']:.3f} m, "
            f"persistence={self.persistence_s:.2f} s, "
            f"TF future fallback<={self.tf_future_fallback_max_gap_s:.3f} s, "
            "cloud_qos=best_effort/keep_last(1)"
        )

    def _merge_held_tracks(self) -> None:
        """Coalesce duplicate fixed-frame tracks without extending lifetime."""
        merge_distance = self.parameters["center_merge_distance"]
        pending = list(self._held_centers)
        merged: list[dict[str, object]] = []
        while pending:
            group = [pending.pop(0)]
            changed = True
            while changed:
                changed = False
                group_centers = [np.asarray(item["center"]) for item in group]
                for pending_index, item in enumerate(pending):
                    center = np.asarray(item["center"])
                    if any(
                        float(np.linalg.norm(center - existing)) <= merge_distance
                        for existing in group_centers
                    ):
                        pending.pop(pending_index)
                        group.append(item)
                        changed = True
                        break
            centers = np.asarray([item["center"] for item in group], dtype=np.float32)
            merged.append(
                {
                    "center": np.median(centers, axis=0).astype(np.float32),
                    "seen": max(float(item["seen"]) for item in group),
                }
            )
        self._held_centers = merged

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
            # A Livox cloud can arrive tens of milliseconds ahead of the most
            # recent odom TF. Fall back only when the latest transform proves
            # this is a small future gap. Past extrapolation, missing frames,
            # and stale/frozen TF remain failures.
            if (
                self.tf_future_fallback_enabled
                and stamp is not None
                and (stamp.sec or stamp.nanosec)
            ):
                try:
                    latest = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        Time(),
                        timeout=Duration(seconds=0.05),
                    )
                    gap_s = future_timestamp_gap_s(
                        stamp.sec,
                        stamp.nanosec,
                        latest.header.stamp.sec,
                        latest.header.stamp.nanosec,
                    )
                    self.tf_last_rejected_gap_s = gap_s
                    if (
                        gap_s is not None
                        and gap_s <= self.tf_future_fallback_max_gap_s
                    ):
                        self.tf_future_fallback_count += 1
                        self.tf_last_fallback_gap_s = gap_s
                        self.tf_last_rejected_gap_s = None
                        now = time.monotonic()
                        if now - self._last_tf_fallback_log > 5.0:
                            self.get_logger().info(
                                "Cone compensation used latest TF for "
                                f"{source_frame}->{target_frame}: "
                                f"future gap={gap_s:.3f} s"
                            )
                            self._last_tf_fallback_log = now
                        return latest.transform
                except TransformException:
                    self.tf_last_rejected_gap_s = None

            self.tf_lookup_failure_count += 1
            now = time.monotonic()
            if now - self._last_tf_warning > 2.0:
                gap_detail = (
                    ""
                    if self.tf_last_rejected_gap_s is None
                    else (
                        f"; latest gap={self.tf_last_rejected_gap_s:.3f} s "
                        f"exceeds {self.tf_future_fallback_max_gap_s:.3f} s"
                    )
                )
                self.get_logger().warning(
                    f"Cone compensation TF {source_frame}->{target_frame} "
                    f"unavailable: {error}{gap_detail}"
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
            self._held_centers = []
            self.held_count = 0
            self._held_centers_base = []
            return []

        current_fixed = self._to_persistence_frame(centers, source_frame, stamp)
        if current_fixed is None:
            self.held_count = len(self._held_centers)
            self._held_centers_base = []
            return []

        # Merge old duplicates first, then match in a fixed frame so robot
        # motion cannot turn one cone into a trail. Very close detections may
        # update the same track; wider matches remain one-to-one so two real
        # cones near each other are not collapsed through a shared track.
        self._merge_held_tracks()
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
                ]
                if candidates:
                    nearest, distance = min(candidates, key=lambda pair: pair[1])
                    can_reuse = (
                        distance <= self.parameters["center_merge_distance"]
                        or nearest not in matched_tracks
                    )
                    if distance <= self.persistence_match_distance and can_reuse:
                        matched_tracks.add(nearest)
                        match = self._held_centers[nearest]
            if match is None:
                self._held_centers.append(
                    {"center": point.astype(np.float32), "seen": now}
                )
            else:
                previous = np.asarray(match["center"], dtype=np.float32)
                match["center"] = (0.5 * previous + 0.5 * point).astype(np.float32)
                match["seen"] = now

        self._merge_held_tracks()

        if not self._held_centers:
            self.held_count = 0
            self._held_centers_base = []
            return []
        active_fixed = np.asarray(
            [item["center"] for item in self._held_centers], dtype=np.float32
        )
        active_base = self._from_persistence_frame(active_fixed, source_frame)
        if active_base is None:
            self.held_count = len(self._held_centers)
            self._held_centers_base = []
            return []

        self._held_centers_base = [
            (float(point[0]), float(point[1])) for point in active_base
        ]

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
        self.held_count = len(self._held_centers)
        return extras

    def _cloud_callback(self, message: PointCloud2) -> None:
        xyz = xyz_from_cloud(message)
        source_frame = message.header.frame_id.lstrip("/") or "base_link"
        expanded, centers, raw_cluster_count = expand_cone_footprints(
            xyz, **self.parameters, return_diagnostics=True
        )
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
        self.raw_cluster_count = raw_cluster_count
        self.detected_count = len(centers)
        self._current_centers_base = list(centers)

    def _publish_status(self) -> None:
        current = sorted(
            self._current_centers_base,
            key=lambda point: math.hypot(point[0], point[1]),
        )
        held_base = sorted(
            self._held_centers_base,
            key=lambda point: math.hypot(point[0], point[1]),
        )
        held_pairs = list(zip(self._held_centers_base, self._held_centers))
        held_pairs.sort(key=lambda pair: math.hypot(*pair[0]))
        held_odom = [
            (float(item["center"][0]), float(item["center"][1]))
            for _, item in held_pairs
        ]

        def rounded(points):
            return [[round(x, 3), round(y, 3)] for x, y in points[:8]]

        message = String()
        message.data = json.dumps(
            {
                "clouds": self.cloud_count,
                "raw_clusters": self.raw_cluster_count,
                "merged_cones": self.detected_count,
                "held_tracks": self.held_count,
                "current_centers_base": rounded(current),
                "held_centers_odom": rounded(held_odom),
                "nearest_current_cone_m": (
                    round(math.hypot(*current[0]), 3) if current else None
                ),
                "nearest_held_cone_m": (
                    round(math.hypot(*held_base[0]), 3) if held_base else None
                ),
                "base_radius_m": self.parameters["base_radius"],
                "physical_base_radius_m": self.parameters["physical_base_radius"],
                "cone_height_m": self.parameters["cone_height"],
                "min_points": self.parameters["min_points"],
                "min_vertical_span_m": self.parameters["min_vertical_span"],
                "center_merge_distance_m": self.parameters["center_merge_distance"],
                "persistence_s": self.persistence_s,
                "tf_future_fallback_enabled": self.tf_future_fallback_enabled,
                "tf_future_fallback_max_gap_s": self.tf_future_fallback_max_gap_s,
                "tf_future_fallbacks": self.tf_future_fallback_count,
                "tf_lookup_failures": self.tf_lookup_failure_count,
                "tf_last_fallback_gap_s": (
                    None
                    if self.tf_last_fallback_gap_s is None
                    else round(self.tf_last_fallback_gap_s, 4)
                ),
                "tf_last_rejected_gap_s": (
                    None
                    if self.tf_last_rejected_gap_s is None
                    else round(self.tf_last_rejected_gap_s, 4)
                ),
            },
            separators=(",", ":"),
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
