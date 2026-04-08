#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import copy
import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import actionlib
import cv2
import message_filters
import numpy as np
import rospy
import rospkg
import tf2_geometry_msgs  # noqa: F401 - required by do_transform_point in rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String, Int16
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class DigitTemplate:
    label: int
    binary: np.ndarray
    contour: np.ndarray


@dataclass
class BoxDetection:
    label: int
    x: float
    y: float
    z: float
    score: float
    hits: int = 1
    last_seen: float = 0.0

    def update(self, x: float, y: float, z: float, score: float, stamp: float) -> None:
        alpha = 1.0 / float(self.hits + 1)
        self.x = (1.0 - alpha) * self.x + alpha * x
        self.y = (1.0 - alpha) * self.y + alpha * y
        self.z = (1.0 - alpha) * self.z + alpha * z
        self.score = min(self.score, score)
        self.hits += 1
        self.last_seen = stamp


class Floor1AutoScanNode:
    def __init__(self) -> None:
        rospy.init_node("floor1_auto_scan", anonymous=False)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        self.package_path = rospkg.RosPack().get_path("me5413_world")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "front_rgbd_optical_frame")
        self.image_topic = rospy.get_param("~image_topic", "/front_rgbd/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/front_rgbd/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/front_rgbd/rgb/camera_info")
        self.global_plan_topic = rospy.get_param("~global_plan_topic", "/move_base/NavfnROS/plan")
        self.process_every_n_frames = max(1, int(rospy.get_param("~process_every_n_frames", 3)))
        self.binary_threshold = int(rospy.get_param("~binary_threshold", 95))
        self.template_corr_threshold = float(rospy.get_param("~template_corr_threshold", 0.40))
        self.template_shape_threshold = float(rospy.get_param("~template_shape_threshold", 0.75))
        self.dedup_radius = float(rospy.get_param("~dedup_radius", 0.90))
        self.min_confirm_hits = int(rospy.get_param("~min_confirm_hits", 2))
        self.min_depth = float(rospy.get_param("~min_depth", 0.45))
        self.max_depth = float(rospy.get_param("~max_depth", 5.50))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 75.0))
        self.goal_retries = int(rospy.get_param("~goal_retries", 2))
        self.dwell_time = float(rospy.get_param("~dwell_time", 0))
        self.path_publish_period = float(rospy.get_param("~path_publish_period", 0.5))
        self.result_file = os.path.expanduser(
            rospy.get_param("~result_file", "~/.ros/me5413_floor1_counts.json")
        )
        self.waypoint_radius = float(rospy.get_param("~waypoint_radius", 0.18))
        self.initialpose_topic = rospy.get_param("~initialpose_topic", "/initialpose")
        self.start_topic = rospy.get_param("~start_topic", "/me5413_world/start_floor1_scan")
        self.start_on_initialpose = bool(rospy.get_param("~start_on_initialpose", False))
        self.initialpose_settle_time = float(rospy.get_param("~initialpose_settle_time", 2.0))
        self.pose_wait_timeout = float(rospy.get_param("~pose_wait_timeout", 30.0))
        self.spawn_topic = rospy.get_param("~spawn_topic", "/rviz_panel/respawn_objects")
        self.spawn_value = int(rospy.get_param("~spawn_value", 1))
        self.start_on_spawn = bool(rospy.get_param("~start_on_spawn", True))
        self.spawn_settle_time = float(rospy.get_param("~spawn_settle_time", 3.0))
        self.goal_margin_x = float(rospy.get_param("~goal_margin_x", 0.8))
        self.goal_margin_y = float(rospy.get_param("~goal_margin_y", 0.8))
        self.default_lane_spacing = float(rospy.get_param("~default_lane_spacing", 2.0))

        self.scan_regions_world = rospy.get_param(
            "~scan_regions",
            [
                {
                    "name": "region_1",
                    "corners": [
                        {"x": 3.3086471558, "y": -1.6212404966},
                        {"x": 3.5671601295, "y": 16.1897811890},
                        {"x": 11.1797428131, "y": 16.3910865784},
                        {"x": 11.0585441589, "y": -1.1625661850},
                    ],
                    "lane_spacing": 2.0,
                },
                {
                    "name": "region_2",
                    "corners": [
                        {"x": 13.7547569275, "y": -1.1491963863},
                        {"x": 13.4040279388, "y": 16.3940525055},
                        {"x": 20.9208736420, "y": 16.7020378113},
                        {"x": 21.2164230347, "y": -0.9532148838},
                    ],
                    "lane_spacing": 2.0,
                },
            ],
        )

        self.camera_info: Optional[CameraInfo] = None
        self.fx = self.fy = self.cx = self.cy = None
        self.frame_counter = 0
        self.occupancy_grid: Optional[OccupancyGrid] = None
        self.latest_global_plan: Optional[Path] = None
        self.travelled_path = Path()
        self.travelled_path.header.frame_id = self.map_frame
        self.confirmed_boxes: List[BoxDetection] = []
        self.status = "booting"
        self.scan_regions: List[dict] = []
        self.waypoints: List[PoseStamped] = []
        self.templates = self._load_digit_templates()
        self.initialpose_received = False
        self.initialpose_walltime = None
        self.spawn_received = False
        self.spawn_walltime = None
        self.start_requested = False
        self.scan_started = False

        self.pub_status = rospy.Publisher("/me5413_world/scan_status", String, queue_size=10, latch=True)
        self.pub_counts = rospy.Publisher("/me5413_world/box_counts", String, queue_size=10, latch=True)
        self.pub_detected_boxes = rospy.Publisher("/me5413_world/detected_boxes", MarkerArray, queue_size=10, latch=True)
        self.pub_scan_waypoints = rospy.Publisher("/me5413_world/scan_waypoints", MarkerArray, queue_size=1, latch=True)
        self.pub_travelled_path = rospy.Publisher("/me5413_world/travelled_path", Path, queue_size=10)

        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self._map_cb, queue_size=1)
        rospy.Subscriber(self.global_plan_topic, Path, self._global_plan_cb, queue_size=1)
        rospy.Subscriber(self.initialpose_topic, PoseWithCovarianceStamped, self._initialpose_cb, queue_size=1)
        rospy.Subscriber(self.start_topic, Bool, self._start_cb, queue_size=1)
        rospy.Subscriber(self.spawn_topic, Int16, self._spawn_cb, queue_size=1)

        rgb_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.15)
        sync.registerCallback(self._rgbd_cb)

        self.path_timer = rospy.Timer(rospy.Duration(self.path_publish_period), self._path_timer_cb)
        self.marker_timer = rospy.Timer(rospy.Duration(1.0), self._marker_timer_cb)

        self._publish_status("waiting_for_move_base")
        rospy.loginfo("[floor1_auto_scan] Waiting for /move_base action server...")
        self.move_base.wait_for_server()
        rospy.loginfo("[floor1_auto_scan] /move_base is ready.")
        self._publish_status("waiting_for_spawn_objects")
        rospy.loginfo("[floor1_auto_scan] Waiting for Respawn Objects trigger. Scan regions use absolute map coordinates.")

    def run(self) -> None:
        rospy.sleep(1.0)

        while not rospy.is_shutdown():
            self._wait_for_start_trigger()
            if rospy.is_shutdown():
                return
            if self._initialize_scan_layout():
                break
            self.spawn_received = False
            self.start_requested = False
            self._publish_status("waiting_for_spawn_objects")
            rospy.logwarn("[floor1_auto_scan] Failed to prepare the scan layout. Please respawn the objects and try again.")

        self._reset_scan_run()
        self.scan_started = True
        self._publish_scan_waypoints()
        self._publish_status("scanning_floor1")
        rospy.loginfo("[floor1_auto_scan] Starting first-floor serpentine scan with %d waypoints.", len(self.waypoints))

        for idx, waypoint in enumerate(self.waypoints):
            if rospy.is_shutdown():
                return
            self._publish_status(f"navigating_waypoint_{idx+1}_of_{len(self.waypoints)}")
            reached = self._goto_with_retries(waypoint, idx)
            rospy.sleep(self.dwell_time)
            if reached:
                rospy.loginfo(
                    "[floor1_auto_scan] Waypoint %d/%d reached. Current counts: %s",
                    idx + 1,
                    len(self.waypoints),
                    self._counts_string(),
                )
            else:
                rospy.logwarn(
                    "[floor1_auto_scan] Waypoint %d/%d skipped after retries. Current counts: %s",
                    idx + 1,
                    len(self.waypoints),
                    self._counts_string(),
                )

        self.move_base.cancel_all_goals()
        self._publish_status("floor1_scan_complete")
        self._publish_counts()
        self._write_result_file()
        self._publish_box_markers()
        rospy.loginfo("[floor1_auto_scan] First-floor scan complete. Final counts: %s", self._counts_string())
        rospy.spin()

    def _reset_scan_run(self) -> None:
        self.confirmed_boxes = []
        self.frame_counter = 0
        self.latest_global_plan = None

        self.travelled_path = Path()
        self.travelled_path.header.frame_id = self.map_frame

        if os.path.isfile(self.result_file):
            try:
                os.remove(self.result_file)
                rospy.loginfo("[floor1_auto_scan] Removed old scan result: %s", self.result_file)
            except Exception as exc:
                rospy.logwarn(
                    "[floor1_auto_scan] Failed to remove old scan result %s: %s",
                    self.result_file,
                    str(exc),
                )

        self.pub_counts.publish(String(data="{}"))
        self.pub_detected_boxes.publish(MarkerArray())
        self.pub_travelled_path.publish(self.travelled_path)

        rospy.loginfo("[floor1_auto_scan] Scan state has been reset for a new run.")

    def _initialize_scan_layout(self) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(self.pose_wait_timeout)
        pose_ok = False
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self._lookup_robot_pose() is not None:
                pose_ok = True
                break
            self._publish_status("waiting_for_map_base_link_tf")
            rospy.sleep(0.2)

        if not pose_ok:
            rospy.logwarn("[floor1_auto_scan] map->base_link is still unavailable after start trigger.")
            return False

        self.scan_regions = self._prepare_scan_regions(self.scan_regions_world)
        self.waypoints = self._build_scan_waypoints()
        if not self.waypoints:
            rospy.logwarn("[floor1_auto_scan] No waypoints were generated from scan_regions.")
            return False

        rospy.loginfo(
            "[floor1_auto_scan] Prepared %d scan regions and %d absolute map-frame waypoints.",
            len(self.scan_regions),
            len(self.waypoints),
        )
        return True

    def _prepare_scan_regions(self, regions: List[dict]) -> List[dict]:
        prepared: List[dict] = []
        for idx, region in enumerate(regions):
            name = str(region.get("name", f"region_{idx + 1}"))
            lane_spacing = float(region.get("lane_spacing", self.default_lane_spacing))

            if "corners" in region:
                points = self._parse_region_corners(region["corners"])
                if len(points) != 4:
                    rospy.logwarn("[floor1_auto_scan] Region %s skipped: corners must contain exactly 4 points.", name)
                    continue
                region_info = self._region_from_corners(name, points, lane_spacing)
            else:
                keys = ("x_min", "x_max", "y_min", "y_max")
                if not all(k in region for k in keys):
                    rospy.logwarn("[floor1_auto_scan] Region %s skipped: missing corners or x/y bounds.", name)
                    continue
                region_info = self._region_from_axis_aligned(name, region, lane_spacing)

            region_info["mirror_start_side"] = bool(region.get("mirror_start_side", False))
            prepared.append(region_info)

        return prepared

    def _parse_region_corners(self, corners_raw: List[dict]) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []
        for item in corners_raw:
            if isinstance(item, dict):
                points.append((float(item["x"]), float(item["y"])))
            else:
                if len(item) != 2:
                    raise ValueError("Each corner must contain exactly two numbers.")
                points.append((float(item[0]), float(item[1])))
        return points

    def _region_from_axis_aligned(self, name: str, region: dict, lane_spacing: float) -> dict:
        x_min = float(region["x_min"])
        x_max = float(region["x_max"])
        y_min = float(region["y_min"])
        y_max = float(region["y_max"])
        corners = [(x_min, y_min), (x_min, y_max), (x_max, y_max), (x_max, y_min)]

        long_len = max(x_max - x_min, y_max - y_min)
        short_len = min(x_max - x_min, y_max - y_min)
        if (x_max - x_min) >= (y_max - y_min):
            u_hat = np.array([1.0, 0.0], dtype=float)
            v_hat = np.array([0.0, 1.0], dtype=float)
            u_min, u_max = x_min, x_max
            v_min, v_max = y_min, y_max
            center = np.array([(x_min + x_max) * 0.5, (y_min + y_max) * 0.5], dtype=float)
            local_pts = []
            for p in corners:
                delta = np.array(p, dtype=float) - center
                local_pts.append((float(np.dot(delta, u_hat)), float(np.dot(delta, v_hat))))
            u_min = min(p[0] for p in local_pts)
            u_max = max(p[0] for p in local_pts)
            v_min = min(p[1] for p in local_pts)
            v_max = max(p[1] for p in local_pts)
        else:
            # Rotate the local frame so that u is always the long axis.
            u_hat = np.array([0.0, 1.0], dtype=float)
            v_hat = np.array([-1.0, 0.0], dtype=float)
            center = np.array([(x_min + x_max) * 0.5, (y_min + y_max) * 0.5], dtype=float)
            local_pts = []
            for p in corners:
                delta = np.array(p, dtype=float) - center
                local_pts.append((float(np.dot(delta, u_hat)), float(np.dot(delta, v_hat))))
            u_min = min(p[0] for p in local_pts)
            u_max = max(p[0] for p in local_pts)
            v_min = min(p[1] for p in local_pts)
            v_max = max(p[1] for p in local_pts)

        return {
            "name": name,
            "lane_spacing": lane_spacing,
            "corners": corners,
            "center": center,
            "u_hat": u_hat,
            "v_hat": v_hat,
            "u_min": u_min,
            "u_max": u_max,
            "v_min": v_min,
            "v_max": v_max,
            "x_min_map": min(p[0] for p in corners),
            "x_max_map": max(p[0] for p in corners),
            "y_min_map": min(p[1] for p in corners),
            "y_max_map": max(p[1] for p in corners),
            "long_len": long_len,
            "short_len": short_len,
        }

    def _region_from_corners(self, name: str, points: List[Tuple[float, float]], lane_spacing: float) -> dict:
        pts = np.array(points, dtype=float)
        center = np.mean(pts, axis=0)

        angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
        ordered = pts[np.argsort(angles)]

        edges = []
        for i in range(4):
            p0 = ordered[i]
            p1 = ordered[(i + 1) % 4]
            vec = p1 - p0
            length = float(np.linalg.norm(vec))
            edges.append((length, vec, p0, p1))

        long_edge = max(edges, key=lambda e: e[0])
        if long_edge[0] < 1e-6:
            raise ValueError(f"Region {name} has degenerate corners.")

        u_hat = long_edge[1] / np.linalg.norm(long_edge[1])
        v_hat = np.array([-u_hat[1], u_hat[0]], dtype=float)

        local_pts = []
        for p in ordered:
            delta = p - center
            local_pts.append((float(np.dot(delta, u_hat)), float(np.dot(delta, v_hat))))

        u_min = min(p[0] for p in local_pts)
        u_max = max(p[0] for p in local_pts)
        v_min = min(p[1] for p in local_pts)
        v_max = max(p[1] for p in local_pts)

        long_len = u_max - u_min
        short_len = v_max - v_min

        return {
            "name": name,
            "lane_spacing": lane_spacing,
            "corners": [(float(p[0]), float(p[1])) for p in ordered],
            "center": center,
            "u_hat": u_hat,
            "v_hat": v_hat,
            "u_min": u_min,
            "u_max": u_max,
            "v_min": v_min,
            "v_max": v_max,
            "x_min_map": float(np.min(ordered[:, 0])),
            "x_max_map": float(np.max(ordered[:, 0])),
            "y_min_map": float(np.min(ordered[:, 1])),
            "y_max_map": float(np.max(ordered[:, 1])),
            "long_len": long_len,
            "short_len": short_len,
        }

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.camera_info = msg
        self.fx = float(msg.K[0])
        self.fy = float(msg.K[4])
        self.cx = float(msg.K[2])
        self.cy = float(msg.K[5])

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.occupancy_grid = msg

    def _global_plan_cb(self, msg: Path) -> None:
        self.latest_global_plan = msg

    def _spawn_cb(self, msg: Int16) -> None:
        if self.scan_started:
            return
        if int(msg.data) != self.spawn_value:
            return

        self.spawn_received = True
        self.spawn_walltime = rospy.get_time()
        if self.start_on_spawn:
            self.start_requested = True

        self._publish_status("spawn_trigger_received")
        rospy.loginfo(
            "[floor1_auto_scan] Respawn Objects detected. Waiting %.1f s for objects to finish spawning.",
            self.spawn_settle_time,
        )

    def _initialpose_cb(self, _msg: PoseWithCovarianceStamped) -> None:
        if self.scan_started:
            rospy.loginfo_throttle(5.0, "[floor1_auto_scan] Ignoring 2D Pose Estimate because the scan is already running.")
            return
        self.initialpose_received = True
        self.initialpose_walltime = rospy.get_time()
        rospy.loginfo_throttle(
            5.0,
            "[floor1_auto_scan] 2D Pose Estimate received, but scan start is controlled by Respawn Objects.",
        )

    def _start_cb(self, msg: Bool) -> None:
        if self.scan_started:
            return
        if msg.data:
            self.start_requested = True
            self._publish_status("manual_start_requested")
            rospy.loginfo("[floor1_auto_scan] Manual floor-1 scan start requested.")

    def _wait_for_start_trigger(self) -> None:
        logged_waiting = False
        while not rospy.is_shutdown():
            if not self.spawn_received:
                self._publish_status("waiting_for_spawn_objects")
                if not logged_waiting:
                    rospy.loginfo("[floor1_auto_scan] Waiting for Respawn Objects trigger.")
                    logged_waiting = True
                rospy.sleep(0.2)
                continue

            if not self.start_requested:
                self._publish_status("waiting_for_manual_start")
                rospy.sleep(0.2)
                continue

            elapsed = rospy.get_time() - float(self.spawn_walltime or rospy.get_time())
            remaining = self.spawn_settle_time - elapsed
            if remaining > 0.0:
                self._publish_status("waiting_for_object_spawn_settle")
                rospy.sleep(min(0.2, remaining))
                continue

            pose = self._lookup_robot_pose()
            if pose is None:
                self._publish_status("waiting_for_map_base_link_tf")
                rospy.sleep(0.2)
                continue

            self._publish_status("ready_to_start_scan")
            return

    def _path_timer_cb(self, _event) -> None:
        pose = self._lookup_robot_pose()
        if pose is None:
            return

        if self.travelled_path.poses:
            last = self.travelled_path.poses[-1].pose.position
            dx = pose.pose.position.x - last.x
            dy = pose.pose.position.y - last.y
            if dx * dx + dy * dy < 0.05 * 0.05:
                self.pub_travelled_path.publish(self.travelled_path)
                return

        self.travelled_path.header.stamp = rospy.Time.now()
        self.travelled_path.poses.append(pose)
        if len(self.travelled_path.poses) > 5000:
            self.travelled_path.poses = self.travelled_path.poses[-5000:]
        self.pub_travelled_path.publish(self.travelled_path)

    def _marker_timer_cb(self, _event) -> None:
        self._publish_box_markers()

    def _rgbd_cb(self, rgb_msg: Image, depth_msg: Image) -> None:
        if self.camera_info is None or self.fx is None:
            return

        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "[floor1_auto_scan] CvBridge error: %s", str(exc))
            return

        if rgb is None or depth is None:
            return

        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(gray, self.binary_threshold, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        stamp = rgb_msg.header.stamp

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 120.0 or area > 14000.0:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < 10 or h < 18:
                continue
            aspect = float(w) / float(h)
            if aspect < 0.15 or aspect > 2.2:
                continue

            pad = 6
            x0 = max(0, x - pad)
            y0 = max(0, y - pad)
            x1 = min(binary.shape[1], x + w + pad)
            y1 = min(binary.shape[0], y + h + pad)
            candidate = binary[y0:y1, x0:x1]
            if candidate.size == 0:
                continue

            label, corr_score, shape_score = self._classify_candidate(candidate)
            if label is None:
                continue
            if corr_score < self.template_corr_threshold:
                continue
            if shape_score > self.template_shape_threshold:
                continue

            u = int(round(x + 0.5 * w))
            v = int(round(y + 0.5 * h))
            depth_value = self._sample_depth(depth, u, v)
            if depth_value is None:
                continue

            point_map = self._pixel_to_map_point(u, v, depth_value, stamp)
            if point_map is None:
                continue

            if not self._point_in_any_region(point_map.point.x, point_map.point.y):
                continue

            if point_map.point.z < 0.0 or point_map.point.z > 1.5:
                continue

            self._upsert_detection(label, point_map.point.x, point_map.point.y, point_map.point.z, 1.0 - corr_score, stamp.to_sec())

    def _classify_candidate(self, candidate_binary: np.ndarray) -> Tuple[Optional[int], float, float]:
        ys, xs = np.where(candidate_binary > 0)
        if len(xs) < 30 or len(ys) < 30:
            return None, 0.0, 1e9

        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        crop = candidate_binary[y0 : y1 + 1, x0 : x1 + 1]
        crop = cv2.resize(crop, (64, 64), interpolation=cv2.INTER_NEAREST)
        crop_float = crop.astype(np.float32) / 255.0
        crop_cnts, _ = cv2.findContours(crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not crop_cnts:
            return None, 0.0, 1e9
        crop_cnt = max(crop_cnts, key=cv2.contourArea)

        best_label = None
        best_corr = -1.0
        best_shape = 1e9

        for templ in self.templates:
            templ_float = templ.binary.astype(np.float32) / 255.0
            corr = self._normalized_correlation(crop_float, templ_float)
            shape = cv2.matchShapes(crop_cnt, templ.contour, cv2.CONTOURS_MATCH_I1, 0.0)
            if corr > best_corr or (abs(corr - best_corr) < 1e-6 and shape < best_shape):
                best_label = templ.label
                best_corr = corr
                best_shape = shape

        return best_label, best_corr, best_shape

    @staticmethod
    def _normalized_correlation(a: np.ndarray, b: np.ndarray) -> float:
        a = a - float(a.mean())
        b = b - float(b.mean())
        denom = float(np.linalg.norm(a) * np.linalg.norm(b))
        if denom < 1e-8:
            return -1.0
        return float(np.sum(a * b) / denom)

    def _sample_depth(self, depth_image: np.ndarray, u: int, v: int) -> Optional[float]:
        h, w = depth_image.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return None
        r = 2
        patch = depth_image[max(0, v - r) : min(h, v + r + 1), max(0, u - r) : min(w, u + r + 1)]
        if patch.size == 0:
            return None
        patch = patch.astype(np.float32)
        if np.nanmax(patch) > 100.0:
            patch *= 0.001
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid >= self.min_depth) & (valid <= self.max_depth)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _pixel_to_map_point(self, u: int, v: int, depth_m: float, stamp: rospy.Time) -> Optional[PointStamped]:
        x = (float(u) - self.cx) * depth_m / self.fx
        y = (float(v) - self.cy) * depth_m / self.fy
        z = depth_m

        pt_cam = PointStamped()
        pt_cam.header.stamp = stamp
        pt_cam.header.frame_id = self.camera_frame
        pt_cam.point.x = x
        pt_cam.point.y = y
        pt_cam.point.z = z

        try:
            tfm = self.tf_buffer.lookup_transform(self.map_frame, self.camera_frame, stamp, rospy.Duration(0.15))
            pt_map = tf2_geometry_msgs.do_transform_point(pt_cam, tfm)
            return pt_map
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[floor1_auto_scan] TF camera->map lookup failed: %s", str(exc))
            return None

    def _upsert_detection(self, label: int, x: float, y: float, z: float, score: float, stamp_sec: float) -> None:
        best_idx = None
        best_dist = 1e9
        for idx, det in enumerate(self.confirmed_boxes):
            dist = math.hypot(det.x - x, det.y - y)
            if dist < best_dist:
                best_dist = dist
                best_idx = idx

        if best_idx is not None and best_dist <= self.dedup_radius:
            det = self.confirmed_boxes[best_idx]
            if det.label == label:
                det.update(x, y, z, score, stamp_sec)
            elif score < det.score * 0.9:
                det.label = label
                det.update(x, y, z, score, stamp_sec)
            return

        self.confirmed_boxes.append(BoxDetection(label=label, x=x, y=y, z=z, score=score, hits=1, last_seen=stamp_sec))

    def _counts(self) -> Dict[int, int]:
        counts: Dict[int, int] = {}
        for det in self.confirmed_boxes:
            if det.hits < self.min_confirm_hits:
                continue
            counts[det.label] = counts.get(det.label, 0) + 1
        return dict(sorted(counts.items(), key=lambda kv: kv[0]))

    def _counts_string(self) -> str:
        counts = self._counts()
        if not counts:
            return "{}"
        return json.dumps(counts, ensure_ascii=False, sort_keys=True)

    def _publish_counts(self) -> None:
        self.pub_counts.publish(String(data=self._counts_string()))

    def _write_result_file(self) -> None:
        result_dir = os.path.dirname(self.result_file)
        if result_dir and not os.path.isdir(result_dir):
            os.makedirs(result_dir, exist_ok=True)

        result = {
            "status": "floor1_scan_complete",
            "generated_at": rospy.Time.now().to_sec(),
            "counts": self._counts(),
            "boxes": [
                {
                    "label": det.label,
                    "x": round(det.x, 3),
                    "y": round(det.y, 3),
                    "z": round(det.z, 3),
                    "hits": det.hits,
                    "score": round(det.score, 4),
                }
                for det in self.confirmed_boxes
                if det.hits >= self.min_confirm_hits
            ],
        }

        with open(self.result_file, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        rospy.loginfo("[floor1_auto_scan] Wrote scan result to %s", self.result_file)

    def _publish_status(self, text: str) -> None:
        self.status = text
        self.pub_status.publish(String(data=text))

    def _publish_box_markers(self) -> None:
        markers = MarkerArray()
        now = rospy.Time.now()
        marker_id = 0
        for det in self.confirmed_boxes:
            if det.hits < self.min_confirm_hits:
                continue

            cube = Marker()
            cube.header.frame_id = self.map_frame
            cube.header.stamp = now
            cube.ns = "box_counter"
            cube.id = marker_id
            marker_id += 1
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = det.x
            cube.pose.position.y = det.y
            cube.pose.position.z = max(0.40, det.z)
            cube.pose.orientation.w = 1.0
            cube.scale.x = 0.70
            cube.scale.y = 0.70
            cube.scale.z = 0.70
            cube.color.a = 0.45
            cube.color.r = 0.10
            cube.color.g = 0.60
            cube.color.b = 1.00
            markers.markers.append(cube)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now
            text.ns = "box_counter_text"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = det.x
            text.pose.position.y = det.y
            text.pose.position.z = max(0.90, det.z + 0.65)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.45
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = f"{det.label} (x{det.hits})"
            markers.markers.append(text)

        self.pub_detected_boxes.publish(markers)

    def _publish_scan_waypoints(self) -> None:
        markers = MarkerArray()
        now = rospy.Time.now()
        for idx, waypoint in enumerate(self.waypoints):
            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = now
            sphere.ns = "scan_waypoints"
            sphere.id = 2 * idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = waypoint.pose
            sphere.scale.x = 0.25
            sphere.scale.y = 0.25
            sphere.scale.z = 0.25
            sphere.color.a = 0.8
            sphere.color.r = 1.0
            sphere.color.g = 0.8
            sphere.color.b = 0.0
            markers.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now
            text.ns = "scan_waypoints_text"
            text.id = 2 * idx + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = copy.deepcopy(waypoint.pose)
            text.pose.position.z += 0.45
            text.scale.z = 0.25
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = str(idx + 1)
            markers.markers.append(text)

        self.pub_scan_waypoints.publish(markers)

    def _lookup_robot_pose(self) -> Optional[PoseStamped]:
        try:
            tfm = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.15))
        except Exception:
            return None

        pose = PoseStamped()
        pose.header.stamp = tfm.header.stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = tfm.transform.translation.x
        pose.pose.position.y = tfm.transform.translation.y
        pose.pose.position.z = tfm.transform.translation.z
        pose.pose.orientation = tfm.transform.rotation
        return pose

    def _goto_with_retries(self, pose: PoseStamped, idx: int) -> bool:
        for attempt in range(self.goal_retries + 1):
            if rospy.is_shutdown():
                return False

            goal = MoveBaseGoal()
            goal.target_pose = copy.deepcopy(pose)
            goal.target_pose.header.stamp = rospy.Time.now()

            current_pose = self._lookup_robot_pose()
            if current_pose is not None:
                goal.target_pose.pose.orientation = current_pose.pose.orientation

            rospy.loginfo(
                "[floor1_auto_scan] Sending waypoint %d/%d attempt %d -> (%.2f, %.2f, keep current heading)",
                idx + 1,
                len(self.waypoints),
                attempt + 1,
                goal.target_pose.pose.position.x,
                goal.target_pose.pose.position.y,
            )

            self.move_base.send_goal(goal)
            finished = self.move_base.wait_for_result(rospy.Duration(self.goal_timeout))
            if not finished:
                rospy.logwarn("[floor1_auto_scan] Goal timeout; cancelling waypoint %d.", idx + 1)
                self.move_base.cancel_goal()
                rospy.sleep(1.0)
                continue

            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                return True

            rospy.logwarn("[floor1_auto_scan] Goal ended with state %d on waypoint %d.", state, idx + 1)
            rospy.sleep(1.0)

        return False

    def _build_scan_waypoints(self) -> List[PoseStamped]:
        waypoints: List[PoseStamped] = []

        for region in self.scan_regions:
            lane = float(region["lane_spacing"])
            u_margin = self.goal_margin_x
            v_margin = self.goal_margin_y

            u_min = float(region["u_min"]) + u_margin
            u_max = float(region["u_max"]) - u_margin
            v_min = float(region["v_min"]) + v_margin
            v_max = float(region["v_max"]) - v_margin

            if u_max <= u_min or v_max <= v_min:
                rospy.logwarn("[floor1_auto_scan] Region %s invalid after margins.", region["name"])
                continue

            # Sweep parallel to the long edge (u-axis), and step across the short edge (v-axis).
            v_values = list(np.arange(v_max, v_min - 1e-6, -lane))
            if len(v_values) == 0:
                continue

            direction_forward = not bool(region.get("mirror_start_side", False))
            for i, v in enumerate(v_values):
                if direction_forward:
                    start_u = u_max
                    end_u = u_min
                    end_yaw = math.atan2(region["u_hat"][1], region["u_hat"][0]) + math.pi
                else:
                    start_u = u_min
                    end_u = u_max
                    end_yaw = math.atan2(region["u_hat"][1], region["u_hat"][0])

                end_xy = self._region_local_to_map(region, end_u, v)
                waypoints.append(self._make_pose(end_xy[0], end_xy[1], end_yaw))

                if i < len(v_values) - 1:
                    next_v = v_values[i + 1]
                    cross_xy = self._region_local_to_map(region, end_u, next_v)
                    cross_yaw = math.atan2(region["v_hat"][1], region["v_hat"][0])
                    if next_v < v:
                        cross_yaw += math.pi
                    waypoints.append(self._make_pose(cross_xy[0], cross_xy[1], cross_yaw))

                direction_forward = not direction_forward

        return waypoints

    def _region_local_to_map(self, region: dict, u: float, v: float) -> Tuple[float, float]:
        center = region["center"]
        p = center + u * region["u_hat"] + v * region["v_hat"]
        return float(p[0]), float(p[1])

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation = Quaternion(*q)
        return pose

    def _point_in_any_region(self, x: float, y: float) -> bool:
        for region in self.scan_regions:
            if float(region["x_min_map"]) <= x <= float(region["x_max_map"]) and float(region["y_min_map"]) <= y <= float(region["y_max_map"]):
                return True
        return False

    @staticmethod
    def _yaw_from_quaternion(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _load_digit_templates(self) -> List[DigitTemplate]:
        templates: List[DigitTemplate] = []
        for label in range(1, 10):
            texture_path = os.path.join(
                self.package_path,
                "models",
                f"number{label}",
                "materials",
                "textures",
                f"number{label}.png",
            )
            image = cv2.imread(texture_path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                rospy.logwarn("[floor1_auto_scan] Failed to load template texture: %s", texture_path)
                continue

            _, inv = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            face_boxes = []
            for c in contours:
                x, y, w, h = cv2.boundingRect(c)
                area = w * h
                if 85000 <= area <= 130000 and w > 250 and h > 250:
                    face_boxes.append((x, y, w, h))
            face_boxes.sort(key=lambda item: (item[1], item[0]))

            for x, y, w, h in face_boxes:
                margin = int(0.08 * min(w, h))
                face = image[y + margin : y + h - margin, x + margin : x + w - margin]
                if face.size == 0:
                    continue
                _, digit_bin = cv2.threshold(face, 180, 255, cv2.THRESH_BINARY_INV)
                ys, xs = np.where(digit_bin > 0)
                if len(xs) == 0 or len(ys) == 0:
                    continue
                x0, x1 = int(xs.min()), int(xs.max())
                y0, y1 = int(ys.min()), int(ys.max())
                digit_bin = digit_bin[y0 : y1 + 1, x0 : x1 + 1]
                digit_bin = cv2.resize(digit_bin, (64, 64), interpolation=cv2.INTER_NEAREST)
                cnts, _ = cv2.findContours(digit_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not cnts:
                    continue
                contour = max(cnts, key=cv2.contourArea)
                templates.append(DigitTemplate(label=label, binary=digit_bin, contour=contour))

        rospy.loginfo("[floor1_auto_scan] Loaded %d digit templates.", len(templates))
        return templates


if __name__ == "__main__":
    try:
        node = Floor1AutoScanNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
