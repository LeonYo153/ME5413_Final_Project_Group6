#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import actionlib
import cv2
import numpy as np
import rospy
import rospkg
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.client import Client as DynClient
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Bool, Header, String
from tf.transformations import quaternion_from_euler


@dataclass
class DigitTemplate:
    label: int
    binary: np.ndarray
    contour: np.ndarray


class Floor2RoomSelectorNode:
    def __init__(self) -> None:
        rospy.init_node("floor2_room_selector", anonymous=False)

        self.bridge = CvBridge()
        self.package_path = rospkg.RosPack().get_path("me5413_world")
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.local_obstacle_client: Optional[DynClient] = None
        self.global_obstacle_client: Optional[DynClient] = None

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.image_topic = rospy.get_param("~image_topic", "/front_rgbd/rgb/image_raw")
        self.scan_topic = rospy.get_param("~scan_topic", "/front/scan")
        self.scan_status_topic = rospy.get_param("~scan_status_topic", "/me5413_world/scan_status")
        self.door_topic = rospy.get_param("~door_topic", "/cmd_unblock")
        self.counts_file = os.path.expanduser(rospy.get_param("~counts_file", "~/.ros/me5413_floor1_counts.json"))
        self.result_file = os.path.expanduser(rospy.get_param("~result_file", "~/.ros/me5413_floor2_result.json"))

        self.goal_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.observe_duration = float(rospy.get_param("~observe_duration", 2.5))
        self.observe_retry = int(rospy.get_param("~observe_retry", 2))
        self.observe_stabilize = float(rospy.get_param("~observe_stabilize", 1.0))
        self.post_arrival_wait = float(rospy.get_param("~post_arrival_wait", 0.5))
        self.open_door_publish_times = int(rospy.get_param("~open_door_publish_times", 5))
        self.open_door_publish_interval = float(rospy.get_param("~open_door_publish_interval", 0.15))
        self.room_count = int(rospy.get_param("~room_count", 4))
        self.room_pitch_y = float(rospy.get_param("~room_pitch_y", 5.0))
        self.room_index_start = int(rospy.get_param("~room_index_start", 1))
        self.room_y_direction = int(rospy.get_param("~room_y_direction", 1))

        self.binary_threshold = int(rospy.get_param("~binary_threshold", 95))
        self.template_corr_threshold = float(rospy.get_param("~template_corr_threshold", 0.40))
        self.template_shape_threshold = float(rospy.get_param("~template_shape_threshold", 0.75))
        self.min_contour_area = float(rospy.get_param("~min_contour_area", 120.0))
        self.max_contour_area = float(rospy.get_param("~max_contour_area", 14000.0))

        self.door_wait_pose_cfg = rospy.get_param(
            "~door_wait_pose",
            {"x": 7.385, "y": -1.126, "yaw": -1.396},
        )
        self.ramp_waypoints_cfg = rospy.get_param(
            "~ramp_waypoints",
            [
                {"name": "ramp_top", "x": 43.129, "y": -1.318, "yaw": 1.420},
            ],
        )
        self.view_anchor_cfg = rospy.get_param(
            "~view_anchor",
            {"x": 31.526, "y": 0.647, "yaw": -3.141},
        )
        self.entry_anchor_cfg = rospy.get_param(
            "~entry_anchor",
            {"x": 28.793, "y": 0.602, "yaw": -3.007},
        )

        self.gap_probe_pose_cfg = rospy.get_param(
            "~gap_probe_pose",
            {"x": 38.8, "y": 10.0, "yaw": -3.056},
        )
        self.left_gap_center = rospy.get_param("~left_gap_center", {"x": 35.8, "y": 7.6})
        self.right_gap_center = rospy.get_param("~right_gap_center", {"x": 35.8, "y": 12.6})
        self.gap_check_radius = float(rospy.get_param("~gap_check_radius", 0.45))
        self.gap_blocked_min_points = int(rospy.get_param("~gap_blocked_min_points", 4))
        self.gap_vote_cycles = int(rospy.get_param("~gap_vote_cycles", 8))
        self.gap_vote_sleep = float(rospy.get_param("~gap_vote_sleep", 0.08))
        self.gap_open_fallback = str(rospy.get_param("~gap_open_fallback", "right")).strip().lower()
        self.blocked_gap_half_depth = float(rospy.get_param("~blocked_gap_half_depth", 0.10))
        self.blocked_gap_half_width = float(rospy.get_param("~blocked_gap_half_width", 0.45))
        self.blocked_gap_point_step = float(rospy.get_param("~blocked_gap_point_step", 0.05))

        self.left_gap_route_cfg = rospy.get_param("~left_gap_route", [])
        self.right_gap_route_cfg = rospy.get_param("~right_gap_route", [])

        self.door_wait_pose = self._cfg_to_pose(self.door_wait_pose_cfg)
        self.gap_probe_pose = self._cfg_to_pose(self.gap_probe_pose_cfg)

        self.ramp_waypoints = []
        for idx, cfg in enumerate(self.ramp_waypoints_cfg):
            name = str(cfg.get("name", f"ramp_waypoint_{idx+1}"))
            pose = self._cfg_to_pose(cfg)
            self.ramp_waypoints.append((name, pose))

        self.left_gap_route = []
        for idx, cfg in enumerate(self.left_gap_route_cfg):
            name = str(cfg.get("name", f"left_gap_waypoint_{idx+1}"))
            self.left_gap_route.append((name, self._cfg_to_pose(cfg)))

        self.right_gap_route = []
        for idx, cfg in enumerate(self.right_gap_route_cfg):
            name = str(cfg.get("name", f"right_gap_waypoint_{idx+1}"))
            self.right_gap_route.append((name, self._cfg_to_pose(cfg)))

        self.view_poses = self._generate_room_poses(self.view_anchor_cfg)
        self.entry_poses = self._generate_room_poses(self.entry_anchor_cfg)

        self.templates = self._load_digit_templates()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_scan: Optional[LaserScan] = None
        self.floor1_done = False
        self.latest_scan_status = ""
        self.latest_counts: Optional[Dict[int, int]] = None
        self.target_label: Optional[int] = None
        self.room_label_map: Dict[int, int] = {}
        self.selected_open_gap: Optional[str] = None

        self.pub_status = rospy.Publisher("/me5413_world/floor2_status", String, queue_size=10, latch=True)
        self.pub_room_map = rospy.Publisher("/me5413_world/floor2_room_map", String, queue_size=10, latch=True)
        self.pub_target_room = rospy.Publisher("/me5413_world/floor2_target_room", String, queue_size=10, latch=True)
        self.pub_door = rospy.Publisher(self.door_topic, Bool, queue_size=1)
        self.pub_blocked_gap_cloud = rospy.Publisher("/me5413_world/blocked_gap_cloud", PointCloud2, queue_size=1, latch=True)

        rospy.Subscriber(self.scan_status_topic, String, self._scan_status_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)

    def run(self) -> None:
        self._publish_status("waiting_for_move_base")
        rospy.loginfo("[floor2_room_selector] Waiting for /move_base action server...")
        self.move_base.wait_for_server()
        rospy.loginfo("[floor2_room_selector] /move_base is ready.")

        self._publish_status("waiting_for_floor1_complete")
        self._wait_for_floor1_complete()
        self.target_label = self._read_least_label()
        rospy.loginfo("[floor2_room_selector] Least-count label from floor 1 is %d.", self.target_label)

        self._publish_status("going_to_door_wait_pose")
        if not self._goto_pose(self.door_wait_pose, "door_wait_pose"):
            self._abort("Failed to reach door_wait_pose.")
            return

        self._publish_status("opening_door")
        self._open_door()

        first_ramp_done = False
        for idx, (name, pose) in enumerate(self.ramp_waypoints):
            if idx == 0:
                rospy.loginfo("[floor2_room_selector] Temporarily disabling obstacle_layer only for door_wait_pose -> %s", name)
                self._set_obstacle_layer_enabled(False)

            self._publish_status(f"going_to_{name}")
            if not self._goto_pose(pose, name):
                self._set_obstacle_layer_enabled(True)
                self._abort(f"Failed to reach {name}.")
                return

            if idx == 0 and not first_ramp_done:
                rospy.loginfo("[floor2_room_selector] Re-enabling obstacle_layer after reaching %s", name)
                self._set_obstacle_layer_enabled(True)
                first_ramp_done = True

        self._publish_status("probing_two_gaps")
        if not self._goto_pose(self.gap_probe_pose, "gap_probe_pose"):
            self._abort("Failed to reach gap_probe_pose.")
            return

        open_gap = self._detect_open_gap()
        if open_gap is None:
            self._abort("Could not determine which gap is open.")
            return

        self.selected_open_gap = open_gap
        blocked_gap = "left" if open_gap == "right" else "right"
        rospy.loginfo("[floor2_room_selector] open_gap=%s blocked_gap=%s", open_gap, blocked_gap)

        self._publish_blocked_gap_cloud(blocked_gap)

        if not self._traverse_gap_route(open_gap):
            self._abort(f"Failed to traverse {open_gap}_gap_route.")
            return

        matched_room = None
        visit_order = self._get_room_visit_order()

        rospy.loginfo("[floor2_room_selector] Room visit order after %s gap: %s",
                    str(self.selected_open_gap), visit_order)

        for room_idx in visit_order:
            self._publish_status(f"observing_room_{room_idx}")
            if not self._goto_pose(self.view_poses[room_idx], f"view_{room_idx}"):
                self._abort(f"Failed to reach view_{room_idx}.")
                return

            label = self._observe_room_label(room_idx)
            if label is None:
                self._abort(f"Could not recognize the room number from view_{room_idx}.")
                return

            self.room_label_map[room_idx] = label
            self.pub_room_map.publish(String(data=json.dumps(self.room_label_map, sort_keys=True)))
            rospy.loginfo("[floor2_room_selector] Room %d appears to be labeled %d.", room_idx, label)

            if int(label) == int(self.target_label):
                matched_room = room_idx
                rospy.loginfo(
                    "[floor2_room_selector] Found target label %d at room %d. Going to entry immediately.",
                    self.target_label,
                    matched_room,
                )
                break

        if matched_room is None:
            self._abort(f"No room matched target label {self.target_label}.")
            return

        self.pub_target_room.publish(
            String(data=json.dumps({"target_label": self.target_label, "room": matched_room, "open_gap": self.selected_open_gap}))
        )
        rospy.loginfo("[floor2_room_selector] Target label %d corresponds to room %d.", self.target_label, matched_room)

        self._publish_status(f"entering_room_{matched_room}")
        if not self._goto_pose(self.entry_poses[matched_room], f"entry_{matched_room}"):
            self._abort(f"Failed to reach entry_{matched_room}.")
            return

        self._write_result_file(matched_room)
        self._publish_status("mission_complete")
        rospy.loginfo("[floor2_room_selector] Mission complete. Entered room %d.", matched_room)
        rospy.spin()

    def _wait_for_floor1_complete(self) -> None:
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.floor1_done:
                return
            rate.sleep()

    def _scan_status_cb(self, msg: String) -> None:
        self.latest_scan_status = msg.data.strip()
        self.floor1_done = self.latest_scan_status == "floor1_scan_complete"

    def _image_cb(self, msg: Image) -> None:
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "[floor2_room_selector] CvBridge error: %s", str(exc))

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _read_least_label(self) -> int:
        deadline = time.time() + 30.0
        while time.time() < deadline and not rospy.is_shutdown():
            if os.path.isfile(self.counts_file):
                try:
                    with open(self.counts_file, "r", encoding="utf-8") as f:
                        data = json.load(f)

                    file_status = str(data.get("status", "")).strip()
                    if file_status != "floor1_scan_complete":
                        rospy.sleep(0.2)
                        continue

                    counts_raw = data.get("counts", {})
                    counts = {int(k): int(v) for k, v in counts_raw.items()}
                    if counts:
                        self.latest_counts = counts
                        least_label = min(counts.items(), key=lambda kv: (kv[1], kv[0]))[0]
                        rospy.loginfo(
                            "[floor2_room_selector] Loaded fresh floor1 result from %s, counts=%s",
                            self.counts_file,
                            json.dumps(counts, sort_keys=True),
                        )
                        return int(least_label)

                except Exception as exc:
                    rospy.logwarn_throttle(
                        2.0,
                        "[floor2_room_selector] Failed to read counts file: %s",
                        str(exc),
                    )

            rospy.sleep(0.2)

        raise RuntimeError(f"Could not read a valid fresh counts result from {self.counts_file}")

    def _open_door(self) -> None:
        msg = Bool(data=True)
        for _ in range(max(1, self.open_door_publish_times)):
            self.pub_door.publish(msg)
            rospy.sleep(self.open_door_publish_interval)
        rospy.loginfo("[floor2_room_selector] Timed door unblock signal sent.")

    def _ensure_obstacle_clients(self) -> None:
        if self.local_obstacle_client is None:
            try:
                self.local_obstacle_client = DynClient("/move_base/local_costmap/obstacle_layer", timeout=2.0)
                rospy.loginfo("[floor2_room_selector] Connected to /move_base/local_costmap/obstacle_layer")
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0,
                    "[floor2_room_selector] Could not connect to local obstacle_layer dynreconf: %s",
                    str(exc),
                )

        if self.global_obstacle_client is None:
            try:
                self.global_obstacle_client = DynClient("/move_base/global_costmap/obstacle_layer", timeout=2.0)
                rospy.loginfo("[floor2_room_selector] Connected to /move_base/global_costmap/obstacle_layer")
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0,
                    "[floor2_room_selector] Could not connect to global obstacle_layer dynreconf: %s",
                    str(exc),
                )

    def _set_obstacle_layer_enabled(self, enabled: bool) -> None:
        self._ensure_obstacle_clients()
        try:
            if self.local_obstacle_client is not None:
                self.local_obstacle_client.update_configuration({"enabled": enabled})
            if self.global_obstacle_client is not None:
                self.global_obstacle_client.update_configuration({"enabled": enabled})
            rospy.loginfo("[floor2_room_selector] obstacle_layer enabled=%s", enabled)
        except Exception as exc:
            rospy.logwarn(
                "[floor2_room_selector] Failed to set obstacle_layer enabled=%s: %s",
                enabled,
                str(exc),
            )

    def _goto_pose(self, pose: PoseStamped, name: str) -> bool:
        goal = MoveBaseGoal()
        goal.target_pose = pose
        rospy.loginfo(
            "[floor2_room_selector] Sending %s -> (%.3f, %.3f, yaw=%.1f deg)",
            name,
            pose.pose.position.x,
            pose.pose.position.y,
            math.degrees(self._yaw_from_quaternion(pose.pose.orientation)),
        )
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(self.goal_timeout))
        if not finished:
            rospy.logwarn("[floor2_room_selector] Goal timeout for %s.", name)
            self.move_base.cancel_goal()
            return False
        state = self.move_base.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn("[floor2_room_selector] Goal %s ended with state %d.", name, state)
            return False
        rospy.sleep(self.post_arrival_wait)
        return True

    def _detect_open_gap(self) -> Optional[str]:
        left_votes = 0
        right_votes = 0

        for _ in range(max(1, self.gap_vote_cycles)):
            scan = self.latest_scan
            if scan is None:
                rospy.sleep(self.gap_vote_sleep)
                continue

            left_hits, right_hits = self._count_gap_hits(scan)
            if left_hits >= self.gap_blocked_min_points:
                left_votes += 1
            if right_hits >= self.gap_blocked_min_points:
                right_votes += 1
            rospy.sleep(self.gap_vote_sleep)

        rospy.loginfo(
            "[floor2_room_selector] Gap votes -> left_blocked_votes=%d right_blocked_votes=%d",
            left_votes,
            right_votes,
        )

        if left_votes > right_votes:
            return "right"
        if right_votes > left_votes:
            return "left"

        if self.gap_open_fallback in ("left", "right"):
            rospy.logwarn("[floor2_room_selector] Gap votes tied. Falling back to %s gap.", self.gap_open_fallback)
            return self.gap_open_fallback
        return None

    def _count_gap_hits(self, scan: LaserScan) -> Tuple[int, int]:
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.map_frame,
                scan.header.frame_id,
                scan.header.stamp,
                rospy.Duration(0.15),
            )
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "[floor2_room_selector] TF lookup failed for scan: %s", str(exc))
            return 0, 0

        tx = tfm.transform.translation.x
        ty = tfm.transform.translation.y
        yaw = self._yaw_from_quaternion(tfm.transform.rotation)
        c = math.cos(yaw)
        s = math.sin(yaw)

        left_hits = 0
        right_hits = 0
        angle = scan.angle_min
        for r in scan.ranges:
            if np.isfinite(r) and scan.range_min <= r <= scan.range_max:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                mx = tx + c * lx - s * ly
                my = ty + s * lx + c * ly

                if (mx - float(self.left_gap_center["x"])) ** 2 + (my - float(self.left_gap_center["y"])) ** 2 <= self.gap_check_radius ** 2:
                    left_hits += 1
                if (mx - float(self.right_gap_center["x"])) ** 2 + (my - float(self.right_gap_center["y"])) ** 2 <= self.gap_check_radius ** 2:
                    right_hits += 1
            angle += scan.angle_increment

        return left_hits, right_hits

    def _publish_blocked_gap_cloud(self, blocked_gap: str) -> None:
        center = self.left_gap_center if blocked_gap == "left" else self.right_gap_center
        pts: List[Tuple[float, float, float]] = []

        xs = np.arange(-self.blocked_gap_half_depth, self.blocked_gap_half_depth + 1e-6, self.blocked_gap_point_step)
        ys = np.arange(-self.blocked_gap_half_width, self.blocked_gap_half_width + 1e-6, self.blocked_gap_point_step)
        for dx in xs:
            for dy in ys:
                pts.append((float(center["x"]) + float(dx), float(center["y"]) + float(dy), 0.35))

        header = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)
        cloud = point_cloud2.create_cloud_xyz32(header, pts)
        self.pub_blocked_gap_cloud.publish(cloud)
        rospy.loginfo("[floor2_room_selector] Published blocked gap cloud for %s gap with %d points.", blocked_gap, len(pts))

    def _traverse_gap_route(self, open_gap: str) -> bool:
        route = self.left_gap_route if open_gap == "left" else self.right_gap_route
        if not route:
            rospy.logwarn("[floor2_room_selector] %s_gap_route is empty. Continuing without gap route.", open_gap)
            return True

        for name, pose in route:
            self._publish_status(f"going_to_{name}")
            if not self._goto_pose(pose, name):
                return False
        return True

    def _get_room_visit_order(self) -> List[int]:
        room_ids = sorted(self.view_poses.keys())

        if self.selected_open_gap == "left":
            return room_ids

        if self.selected_open_gap == "right":
            return list(reversed(room_ids))

        return room_ids

    def _observe_room_label(self, room_idx: int) -> Optional[int]:
        for attempt in range(self.observe_retry + 1):
            rospy.sleep(self.observe_stabilize)
            votes: Dict[int, float] = {}
            count_votes: Dict[int, int] = {}
            deadline = time.time() + self.observe_duration
            while time.time() < deadline and not rospy.is_shutdown():
                image = None if self.latest_image is None else self.latest_image.copy()
                if image is None:
                    rospy.sleep(0.05)
                    continue
                label, confidence = self._detect_best_label(image)
                if label is not None:
                    votes[label] = votes.get(label, 0.0) + confidence
                    count_votes[label] = count_votes.get(label, 0) + 1
                rospy.sleep(0.05)

            if votes:
                best_label = max(votes.items(), key=lambda kv: (kv[1], count_votes.get(kv[0], 0), -kv[0]))[0]
                rospy.loginfo(
                    "[floor2_room_selector] Observation attempt %d for room %d votes=%s -> label=%d",
                    attempt + 1,
                    room_idx,
                    json.dumps({str(k): round(v, 3) for k, v in votes.items()}, sort_keys=True),
                    best_label,
                )
                return int(best_label)

            rospy.logwarn(
                "[floor2_room_selector] No valid digit detected at view_%d on attempt %d.",
                room_idx,
                attempt + 1,
            )
        return None

    def _detect_best_label(self, rgb: np.ndarray) -> Tuple[Optional[int], float]:
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(gray, self.binary_threshold, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_label = None
        best_conf = -1.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_contour_area or area > self.max_contour_area:
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

            confidence = 0.8 * corr_score + 0.2 * max(0.0, 1.0 - shape_score)
            if confidence > best_conf:
                best_conf = confidence
                best_label = label

        if best_label is None:
            return None, 0.0
        return int(best_label), float(best_conf)

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
                rospy.logwarn("[floor2_room_selector] Failed to load template texture: %s", texture_path)
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

        rospy.loginfo("[floor2_room_selector] Loaded %d digit templates.", len(templates))
        return templates

    def _write_result_file(self, target_room: int) -> None:
        result_dir = os.path.dirname(self.result_file)
        if result_dir and not os.path.isdir(result_dir):
            os.makedirs(result_dir, exist_ok=True)
        payload = {
            "floor1_counts": self.latest_counts or {},
            "target_label": self.target_label,
            "room_label_map": self.room_label_map,
            "target_room": target_room,
            "selected_open_gap": self.selected_open_gap,
            "door_wait_pose": self.door_wait_pose_cfg,
            "ramp_waypoints": self.ramp_waypoints_cfg,
        }
        with open(self.result_file, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        rospy.loginfo("[floor2_room_selector] Wrote mission result to %s", self.result_file)

    def _publish_status(self, text: str) -> None:
        self.pub_status.publish(String(data=text))

    def _abort(self, reason: str) -> None:
        rospy.logerr("[floor2_room_selector] %s", reason)
        self._set_obstacle_layer_enabled(True)
        self._publish_status("mission_failed")
        self.move_base.cancel_all_goals()

    def _cfg_to_pose(self, cfg: Dict[str, float]) -> PoseStamped:
        return self._make_pose(float(cfg["x"]), float(cfg["y"]), float(cfg["yaw"]))

    def _generate_room_poses(self, anchor_cfg: Dict[str, float]) -> Dict[int, PoseStamped]:
        poses: Dict[int, PoseStamped] = {}
        base_x = float(anchor_cfg["x"])
        base_y = float(anchor_cfg["y"])
        base_yaw = float(anchor_cfg["yaw"])
        for idx in range(self.room_count):
            room_idx = self.room_index_start + idx
            y = base_y + self.room_y_direction * idx * self.room_pitch_y
            poses[room_idx] = self._make_pose(base_x, y, base_yaw)
        return poses

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

    @staticmethod
    def _yaw_from_quaternion(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


if __name__ == "__main__":
    try:
        node = Floor2RoomSelectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("[floor2_room_selector] Fatal error: %s", str(exc))
