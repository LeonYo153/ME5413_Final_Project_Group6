#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray



@dataclass
class TrackedObstacle:
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    hits: int
    last_stamp: float
    birth_x: float
    birth_y: float
    dynamic_hits: int = 0

    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    def displacement_from_birth(self) -> float:
        return math.hypot(self.x - self.birth_x, self.y - self.birth_y)

class DynamicObstaclePredictorNode:
    def __init__(self) -> None:
        rospy.init_node("dynamic_obstacle_predictor", anonymous=False)

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.scan_topic = rospy.get_param("~scan_topic", "/front/scan")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.predicted_cloud_topic = rospy.get_param(
            "~predicted_cloud_topic", "/me5413_world/predicted_obstacle_cloud"
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/me5413_world/dynamic_obstacle_markers"
        )

        self.cluster_tolerance = float(rospy.get_param("~cluster_tolerance", 0.35))
        self.min_cluster_points = int(rospy.get_param("~min_cluster_points", 3))
        self.track_match_distance = float(rospy.get_param("~track_match_distance", 0.90))
        self.track_timeout = float(rospy.get_param("~track_timeout", 1.0))
        self.min_track_hits = int(rospy.get_param("~min_track_hits", 2))
        self.prediction_horizon = float(rospy.get_param("~prediction_horizon", 1.2))
        self.prediction_dt = float(rospy.get_param("~prediction_dt", 0.2))
        self.default_radius = float(rospy.get_param("~default_radius", 0.30))
        self.safety_margin = float(rospy.get_param("~safety_margin", 0.18))
        self.max_range = float(rospy.get_param("~max_range", 5.0))
        self.min_range = float(rospy.get_param("~min_range", 0.10))
        self.static_occ_threshold = int(rospy.get_param("~static_occ_threshold", 50))
        self.static_filter_radius = float(rospy.get_param("~static_filter_radius", 0.18))
        self.min_speed_for_prediction = float(rospy.get_param("~min_speed_for_prediction", 0.22))
        self.cloud_disc_step = float(rospy.get_param("~cloud_disc_step", 0.12))
        self.marker_lifetime = float(rospy.get_param("~marker_lifetime", 0.4))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_data: Optional[np.ndarray] = None
        self.tracks: Dict[int, TrackedObstacle] = {}
        self.next_track_id = 1

        self.pub_cloud = rospy.Publisher(self.predicted_cloud_topic, PointCloud2, queue_size=1)
        self.pub_markers = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)

        rospy.loginfo(
            "[dynamic_obstacle_predictor] Publishing predicted obstacles to %s and markers to %s",
            self.predicted_cloud_topic,
            self.marker_topic,
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_data = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))

    def _scan_cb(self, msg: LaserScan) -> None:
        if self.map_msg is None or self.map_data is None:
            return

        points = self._scan_to_dynamic_points(msg)
        detections = self._cluster_points(points)
        stamp_sec = msg.header.stamp.to_sec() if msg.header.stamp != rospy.Time() else rospy.Time.now().to_sec()
        self._update_tracks(detections, stamp_sec)
        self._prune_tracks(stamp_sec)
        self._publish_outputs(msg.header.stamp)

    def _scan_to_dynamic_points(self, scan: LaserScan) -> np.ndarray:
        transform = self._lookup_transform(scan.header.frame_id, scan.header.stamp)
        if transform is None:
            return np.zeros((0, 2), dtype=np.float32)

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(ranges.shape[0], dtype=np.float32) * scan.angle_increment
        valid = np.isfinite(ranges)
        valid &= ranges >= max(self.min_range, float(scan.range_min))
        valid &= ranges <= min(self.max_range, float(scan.range_max))
        if not np.any(valid):
            return np.zeros((0, 2), dtype=np.float32)

        rs = ranges[valid]
        ang = angles[valid]
        pts = np.zeros((rs.shape[0], 3), dtype=np.float32)
        pts[:, 0] = rs * np.cos(ang)
        pts[:, 1] = rs * np.sin(ang)

        pts_map = self._transform_points(pts, transform)
        if pts_map.shape[0] == 0:
            return np.zeros((0, 2), dtype=np.float32)

        keep = []
        for p in pts_map:
            if not self._is_static_map_obstacle(float(p[0]), float(p[1])):
                keep.append((float(p[0]), float(p[1])))
        if not keep:
            return np.zeros((0, 2), dtype=np.float32)
        return np.asarray(keep, dtype=np.float32)

    def _lookup_transform(self, source_frame: str, stamp: rospy.Time):
        try:
            return self.tf_buffer.lookup_transform(self.map_frame, source_frame, stamp, rospy.Duration(0.15))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[dynamic_obstacle_predictor] TF %s->%s failed: %s", source_frame, self.map_frame, str(exc))
            return None

    @staticmethod
    def _transform_points(points_xyz: np.ndarray, transform) -> np.ndarray:
        trans = transform.transform.translation
        rot = transform.transform.rotation
        q = [rot.x, rot.y, rot.z, rot.w]
        rot_mat = quaternion_matrix(q)[:3, :3]
        t = np.array([trans.x, trans.y, trans.z], dtype=np.float32)
        return (points_xyz @ rot_mat.T) + t

    def _is_static_map_obstacle(self, x: float, y: float) -> bool:
        if self.map_msg is None or self.map_data is None:
            return False
        info = self.map_msg.info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return True
        radius_cells = max(0, int(math.ceil(self.static_filter_radius / info.resolution)))
        x0 = max(0, mx - radius_cells)
        x1 = min(info.width - 1, mx + radius_cells)
        y0 = max(0, my - radius_cells)
        y1 = min(info.height - 1, my + radius_cells)
        patch = self.map_data[y0 : y1 + 1, x0 : x1 + 1]
        return bool(np.any(patch >= self.static_occ_threshold))

    def _cluster_points(self, points_xy: np.ndarray) -> List[Tuple[float, float, float]]:
        n = points_xy.shape[0]
        if n == 0:
            return []

        visited = np.zeros(n, dtype=bool)
        clusters: List[Tuple[float, float, float]] = []
        tol2 = self.cluster_tolerance * self.cluster_tolerance

        for i in range(n):
            if visited[i]:
                continue
            queue = [i]
            visited[i] = True
            members = []
            while queue:
                idx = queue.pop()
                members.append(idx)
                diff = points_xy - points_xy[idx]
                near = np.where((diff[:, 0] ** 2 + diff[:, 1] ** 2) <= tol2)[0]
                for j in near:
                    if not visited[j]:
                        visited[j] = True
                        queue.append(int(j))
            if len(members) < self.min_cluster_points:
                continue
            cluster_pts = points_xy[members]
            centroid = np.mean(cluster_pts, axis=0)
            dists = np.linalg.norm(cluster_pts - centroid, axis=1)
            radius = max(self.default_radius, float(np.percentile(dists, 90)) + self.safety_margin)
            clusters.append((float(centroid[0]), float(centroid[1]), float(radius)))
        return clusters

    def _update_tracks(self, detections: List[Tuple[float, float, float]], stamp_sec: float) -> None:
        track_ids = list(self.tracks.keys())
        used_tracks = set()
        used_det = set()
        pairs: List[Tuple[float, int, int]] = []

        for det_idx, (dx, dy, _dr) in enumerate(detections):
            for tid in track_ids:
                track = self.tracks[tid]
                dist = math.hypot(track.x - dx, track.y - dy)
                if dist <= self.track_match_distance:
                    pairs.append((dist, tid, det_idx))
        pairs.sort(key=lambda item: item[0])

        for dist, tid, det_idx in pairs:
            if tid in used_tracks or det_idx in used_det:
                continue
            used_tracks.add(tid)
            used_det.add(det_idx)
            dx, dy, dr = detections[det_idx]
            track = self.tracks[tid]
            dt = max(1e-3, stamp_sec - track.last_stamp)
            meas_vx = (dx - track.x) / dt
            meas_vy = (dy - track.y) / dt
            alpha = 0.45
            track.vx = (1.0 - alpha) * track.vx + alpha * meas_vx
            track.vy = (1.0 - alpha) * track.vy + alpha * meas_vy

            speed = math.hypot(track.vx, track.vy)

            if speed < 0.12:
                track.vx = 0.0
                track.vy = 0.0
                speed = 0.0

            track.x = dx
            track.y = dy
            track.radius = max(track.radius * 0.7 + dr * 0.3, self.default_radius)
            track.hits += 1
            track.last_stamp = stamp_sec

            if speed >= self.min_speed_for_prediction and track.displacement_from_birth() >= 0.45:
                track.dynamic_hits += 1
            else:
                track.dynamic_hits = 0

        for det_idx, (dx, dy, dr) in enumerate(detections):
            if det_idx in used_det:
                continue
            self.tracks[self.next_track_id] = TrackedObstacle(
                track_id=self.next_track_id,
                x=dx,
                y=dy,
                vx=0.0,
                vy=0.0,
                radius=max(dr, self.default_radius),
                hits=1,
                last_stamp=stamp_sec,
                birth_x=dx,
                birth_y=dy,
                dynamic_hits=0,
            )
            self.next_track_id += 1

    def _prune_tracks(self, now_sec: float) -> None:
        stale = [tid for tid, track in self.tracks.items() if (now_sec - track.last_stamp) > self.track_timeout]
        for tid in stale:
            self.tracks.pop(tid, None)

    def _publish_outputs(self, stamp: rospy.Time) -> None:
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        points_xyz: List[Tuple[float, float, float]] = []
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        marker_id = 0
        lifetime = rospy.Duration(self.marker_lifetime)

        for track in self.tracks.values():
            if track.hits < self.min_track_hits:
                continue

            if track.dynamic_hits < 5:
                continue

            steps = int(max(1, round(self.prediction_horizon / max(self.prediction_dt, 1e-3))))
            samples = [i * self.prediction_dt for i in range(steps + 1)]

            traj_pts = []
            for t in samples:
                px = track.x + track.vx * t
                py = track.y + track.vy * t
                traj_pts.append((px, py))
                points_xyz.extend(self._disc_points(px, py, max(track.radius, self.default_radius)))

            color = ColorRGBA(1.0, 0.2, 0.1, 0.95)

            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = stamp
            sphere.ns = "dynamic_tracks"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = track.x
            sphere.pose.position.y = track.y
            sphere.pose.position.z = 0.25
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 2.0 * max(track.radius, self.default_radius)
            sphere.scale.y = 2.0 * max(track.radius, self.default_radius)
            sphere.scale.z = 0.20
            sphere.color = color
            sphere.lifetime = lifetime
            markers.markers.append(sphere)

            line = Marker()
            line.header.frame_id = self.map_frame
            line.header.stamp = stamp
            line.ns = "dynamic_prediction"
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.08
            line.color = ColorRGBA(1.0, 0.8, 0.1, 0.95)
            line.lifetime = lifetime
            for px, py in traj_pts:
                p = type(sphere.pose.position)()
                p.x = px
                p.y = py
                p.z = 0.35
                line.points.append(p)
            markers.markers.append(line)

            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = stamp
            arrow.ns = "dynamic_velocity"
            arrow.id = marker_id
            marker_id += 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.scale.x = 0.07
            arrow.scale.y = 0.14
            arrow.scale.z = 0.14
            arrow.color = ColorRGBA(0.1, 1.0, 0.2, 0.95)
            arrow.lifetime = lifetime
            p0 = type(sphere.pose.position)()
            p0.x = track.x
            p0.y = track.y
            p0.z = 0.45
            p1 = type(sphere.pose.position)()
            p1.x = track.x + track.vx * min(self.prediction_horizon, 0.8)
            p1.y = track.y + track.vy * min(self.prediction_horizon, 0.8)
            p1.z = 0.45
            arrow.points = [p0, p1]
            markers.markers.append(arrow)

        header = scan_header = type('obj', (), {})()
        cloud = pc2.create_cloud_xyz32(self._make_header(stamp), points_xyz)
        self.pub_cloud.publish(cloud)
        self.pub_markers.publish(markers)

    def _make_header(self, stamp: rospy.Time):
        from std_msgs.msg import Header
        return Header(stamp=stamp, frame_id=self.map_frame)

    def _disc_points(self, x: float, y: float, radius: float) -> List[Tuple[float, float, float]]:
        pts: List[Tuple[float, float, float]] = []
        step = max(0.06, self.cloud_disc_step)
        rs = np.arange(0.0, radius + 1e-6, step)
        for r in rs:
            if r < 1e-6:
                pts.append((x, y, 0.20))
                continue
            circumference = max(6, int(math.ceil(2.0 * math.pi * r / step)))
            for k in range(circumference):
                th = 2.0 * math.pi * float(k) / float(circumference)
                pts.append((x + r * math.cos(th), y + r * math.sin(th), 0.20))
        return pts


if __name__ == "__main__":
    try:
        node = DynamicObstaclePredictorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
