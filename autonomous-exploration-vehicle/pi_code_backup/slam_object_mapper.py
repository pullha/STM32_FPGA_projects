#!/usr/bin/env python3
import json
import math
import os
import sys
import threading
import time
from collections import defaultdict

sys.path.append("/home/pi/ros_genpy")

import cv2
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from slam_geometry import associate_lidar, base_to_map, camera_bearing, raw_to_base_angle

# Use only the near-field range needed by the narrow test track for SLAM.
SLAM_MIN_RANGE_M = 0.05
SLAM_MAX_RANGE_M = 0.50


OBJECTS = {
    0x01: {"name": "RED SPHERE", "bgr": (40, 40, 230), "rgba": (1.0, 0.05, 0.05, 1.0)},
    0x02: {"name": "GREEN CUBE", "bgr": (40, 200, 40), "rgba": (0.05, 0.9, 0.1, 1.0)},
    0x03: {"name": "BLUE PRISM", "bgr": (230, 80, 40), "rgba": (0.05, 0.25, 1.0, 1.0)},
}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0

    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


class SimpleSlamObjectMapper:
    def __init__(self):
        self.resolution = rospy.get_param("~resolution", 0.05)
        self.size = int(rospy.get_param("~size", 600))
        self.origin_x = -self.size * self.resolution / 2.0
        self.origin_y = -self.size * self.resolution / 2.0
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.laser_frame = rospy.get_param("~laser_frame", "laser_map")
        self.front_deg = float(rospy.get_param("~front_deg", -97.0))
        self.angle_sign = float(rospy.get_param("~angle_sign", -1.0))
        if self.angle_sign not in (-1.0, 1.0):
            raise ValueError("angle_sign must be -1 or +1")
        self.camera_x = float(rospy.get_param("~camera_x_m", 0.0))
        self.camera_y = float(rospy.get_param("~camera_y_m", 0.0))
        self.camera_yaw = math.radians(float(rospy.get_param("~camera_yaw_deg", 0.0)))
        self.sensor_timeout = float(rospy.get_param("~sensor_timeout", 0.5))
        self.confirm_frames = int(rospy.get_param("~confirm_frames", 5))
        self.camera_fov_rad = math.radians(float(rospy.get_param("~camera_fov_deg", 62.0)))
        self.enable_camera = bool(rospy.get_param("~enable_camera", True))
        self.output_dir = rospy.get_param("~output_dir", "/home/pi/slam_outputs")
        os.makedirs(self.output_dir, exist_ok=True)
        self.raw_scan_file = open(os.path.join(self.output_dir, "raw_scans.jsonl"), "w", buffering=1)

        self.logodds = np.zeros((self.size, self.size), dtype=np.float32)
        self.pose = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.latest_scan = None
        self.latest_scan_lock = threading.RLock()
        self.last_scan_time = 0.0
        self.last_camera_time = 0.0
        self.last_processed_frame_time = 0.0
        self.scan_count = 0
        self.object_positions = {}
        self.object_metadata = {}
        self.current_detections = []
        self.object_counts = defaultdict(int)
        self.last_save_time = 0.0
        self.last_camera_debug_time = 0.0
        self.frame_index = 0

        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/object_markers", MarkerArray, queue_size=1, latch=True)
        self.aligned_scan_pub = rospy.Publisher("/scan/map", LaserScan, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

        self.camera_thread = None
        if self.enable_camera:
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()

    def world_to_cell(self, x, y):
        cx = math.floor((x - self.origin_x) / self.resolution)
        cy = math.floor((y - self.origin_y) / self.resolution)
        return cx, cy

    def cell_inside(self, cx, cy):
        return 0 <= cx < self.size and 0 <= cy < self.size

    def scan_points(self, scan, step=5):
        points = []
        for i in range(0, len(scan.ranges), step):
            r = scan.ranges[i]
            if (not math.isfinite(r) or r < scan.range_min or r > scan.range_max
                    or r < SLAM_MIN_RANGE_M or r > SLAM_MAX_RANGE_M):
                continue
            angle = raw_to_base_angle(scan.angle_min + i * scan.angle_increment,
                                      self.front_deg, self.angle_sign)
            points.append((float(r), float(angle)))
        return points

    def scan_match(self, points):
        occupied_known = np.count_nonzero(self.logodds > 0.7)
        if occupied_known < 80 or not points:
            return self.pose

        best_pose = self.pose.copy()
        best_score = -1e9
        dx_candidates = (0.0, -0.09, 0.09, -0.18, 0.18)
        dy_candidates = (0.0, -0.06, 0.06, -0.12, 0.12)
        dt_candidates = (0.0, -0.06, 0.06, -0.12, 0.12, -0.24, 0.24)
        polar = np.asarray(points[::2], dtype=np.float32)
        lx = polar[:, 0] * np.cos(polar[:, 1])
        ly = polar[:, 0] * np.sin(polar[:, 1])

        for dx in dx_candidates:
            for dy in dy_candidates:
                for dt in dt_candidates:
                    x = self.pose[0] + dx
                    y = self.pose[1] + dy
                    theta = self.pose[2] + dt
                    cos_t = math.cos(theta)
                    sin_t = math.sin(theta)
                    wx = x + cos_t * lx - sin_t * ly
                    wy = y + sin_t * lx + cos_t * ly
                    cx = np.floor((wx - self.origin_x) / self.resolution).astype(np.int32)
                    cy = np.floor((wy - self.origin_y) / self.resolution).astype(np.int32)
                    valid = (cx >= 0) & (cx < self.size) & (cy >= 0) & (cy < self.size)
                    score = float(self.logodds[cy[valid], cx[valid]].sum())
                    score -= 0.01 * (abs(dx) + abs(dy) + abs(dt))

                    if score > best_score:
                        best_score = score
                        best_pose[:] = (x, y, theta)

        best_pose[2] = math.atan2(math.sin(best_pose[2]), math.cos(best_pose[2]))
        return best_pose

    def integrate_scan(self, points):
        robot_cx, robot_cy = self.world_to_cell(self.pose[0], self.pose[1])
        if not self.cell_inside(robot_cx, robot_cy):
            return

        cos_t = math.cos(self.pose[2])
        sin_t = math.sin(self.pose[2])

        for r, angle in points:
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            wx = self.pose[0] + cos_t * lx - sin_t * ly
            wy = self.pose[1] + sin_t * lx + cos_t * ly
            end_cx, end_cy = self.world_to_cell(wx, wy)
            if not self.cell_inside(end_cx, end_cy):
                continue

            ray = list(bresenham(robot_cx, robot_cy, end_cx, end_cy))
            for cx, cy in ray[:-1]:
                self.logodds[cy, cx] = clamp(self.logodds[cy, cx] - 0.18, -3.0, 3.5)
            self.logodds[end_cy, end_cx] = clamp(self.logodds[end_cy, end_cx] + 0.85, -3.0, 3.5)

    def scan_callback(self, scan):
        self.raw_scan_file.write(json.dumps({
            "stamp": scan.header.stamp.to_sec(),
            "angle_min": scan.angle_min,
            "angle_increment": scan.angle_increment,
            "range_min": scan.range_min,
            "range_max": scan.range_max,
            "ranges": list(scan.ranges),
        }, allow_nan=True) + "\n")
        points = self.scan_points(scan)
        if not points:
            return
        with self.latest_scan_lock:
            self.pose = self.scan_match(points)
            self.integrate_scan(points)
            # Commit scan and matching pose together for camera projections.
            self.latest_scan = scan
            self.last_scan_time = time.monotonic()
            self.scan_count += 1
            self.publish_aligned_scan(scan)
            self.publish_tf(scan.header.stamp)
            now = time.monotonic()
            if now - self.last_save_time > 1.0:
                self.publish_map()
                self.publish_markers()
                self.save_map_png()
                self.last_save_time = now

    def publish_aligned_scan(self, scan):
        msg = LaserScan()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = self.laser_frame
        count = len(scan.ranges)
        msg.angle_min = -math.pi
        msg.angle_increment = 2.0 * math.pi / count
        msg.angle_max = msg.angle_min + (count - 1) * msg.angle_increment
        msg.range_min, msg.range_max = scan.range_min, scan.range_max
        msg.scan_time = scan.scan_time
        msg.ranges = [float("inf")] * count
        for i, distance in enumerate(scan.ranges):
            angle = raw_to_base_angle(scan.angle_min + i * scan.angle_increment,
                                      self.front_deg, self.angle_sign)
            index = int(round((angle - msg.angle_min) / msg.angle_increment)) % count
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                msg.ranges[index] = min(msg.ranges[index], distance)
        self.aligned_scan_pub.publish(msg)

    def publish_tf(self, stamp):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, float(self.pose[2]))
        self.tf_broadcaster.sendTransform(
            (float(self.pose[0]), float(self.pose[1]), 0.0),
            quat,
            stamp,
            self.base_frame,
            self.map_frame,
        )
        self.tf_broadcaster.sendTransform(
            (0.0, 0.0, 0.12),
            tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
            stamp,
            self.laser_frame,
            self.base_frame,
        )

    def occupancy_data(self):
        occ = np.full(self.logodds.shape, -1, dtype=np.int8)
        occ[self.logodds < -0.35] = 0
        occ[self.logodds > 0.75] = 100
        # ROS uses bottom-left origin, row-major cells. Only images need flipping.
        return occ.reshape(-1).tolist()

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.info.map_load_time = rospy.Time.now()
        msg.info.resolution = self.resolution
        msg.info.width = self.size
        msg.info.height = self.size
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.occupancy_data()
        self.map_pub.publish(msg)

    def marker_color(self, obj_id):
        r, g, b, a = OBJECTS[obj_id]["rgba"]
        return ColorRGBA(r=r, g=g, b=b, a=a)

    def publish_markers(self):
        markers = MarkerArray()
        now = rospy.Time.now()

        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        robot = Marker()
        robot.header.frame_id = self.map_frame
        robot.header.stamp = now
        robot.ns = "robot"
        robot.id = 0
        robot.type = Marker.ARROW
        robot.action = Marker.ADD
        robot.pose.position.x = float(self.pose[0])
        robot.pose.position.y = float(self.pose[1])
        robot.pose.orientation.z = math.sin(float(self.pose[2]) / 2.0)
        robot.pose.orientation.w = math.cos(float(self.pose[2]) / 2.0)
        robot.scale.x = 0.35
        robot.scale.y = 0.08
        robot.scale.z = 0.08
        robot.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        markers.markers.append(robot)

        for obj_id, (x, y, stamp) in self.object_positions.items():
            info = OBJECTS[obj_id]
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = now
            marker.ns = "objects"
            marker.id = obj_id
            marker.type = Marker.SPHERE if obj_id == 0x01 else Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.28
            marker.scale.y = 0.28
            marker.scale.z = 0.28
            marker.color = self.marker_color(obj_id)
            markers.markers.append(marker)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = now
            label.ns = "labels"
            label.id = obj_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(x)
            label.pose.position.y = float(y)
            label.pose.position.z = 0.55
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color = self.marker_color(obj_id)
            label.text = info["name"] + " (estimated)"
            markers.markers.append(label)

        self.marker_pub.publish(markers)

    def detect_objects(self, frame):
        frame_h, frame_w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = {
            0x01: cv2.bitwise_or(
                cv2.inRange(hsv, (0, 120, 100), (8, 255, 255)),
                cv2.inRange(hsv, (172, 120, 100), (180, 255, 255)),
            ),
            0x02: cv2.inRange(hsv, (35, 45, 35), (90, 255, 255)),
            0x03: cv2.inRange(hsv, (90, 45, 35), (140, 255, 255)),
        }

        candidates_by_id = {}
        kernel = np.ones((5, 5), np.uint8)
        for obj_id, mask in masks.items():
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:10]

            candidates = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 700:
                    continue

                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                circularity = 0.0 if peri <= 0 else 4.0 * math.pi * area / (peri * peri)
                x, y, w, h = cv2.boundingRect(contour)
                aspect = w / float(h) if h > 0 else 0.0
                edge_touch = x <= 6 or y <= 6 or (x + w) >= frame_w - 6 or (y + h) >= frame_h - 6
                max_side = max(w, h)
                min_side = min(w, h)

                shape_ok = False
                score = area
                if obj_id == 0x01:
                    # The red target is a small golf-ball-like sphere. Reject edge-clipped
                    # red/brown background regions, then prefer compact circular contours.
                    shape_ok = (
                        not edge_touch
                        and 0.55 <= aspect <= 1.65
                        and 22 <= min_side
                        and max_side <= 170
                        and 700 <= area <= 18000
                        and circularity >= 0.26
                    )
                    score = circularity * 1200.0 + area * 0.02 - abs(w - h) * 2.0
                elif obj_id == 0x02:
                    shape_ok = not edge_touch and 4 <= len(approx) <= 7
                elif obj_id == 0x03:
                    shape_ok = not edge_touch and 3 <= len(approx) <= 6

                if shape_ok:
                    candidates.append(
                        {
                            "obj_id": obj_id,
                            "area": area,
                            "cx": x + w / 2.0,
                            "cy": y + h / 2.0,
                            "bbox": (x, y, w, h),
                            "score": score,
                        }
                    )

            if not candidates:
                self.object_counts[obj_id] = 0
            candidates_by_id[obj_id] = candidates

        selected = {}
        for obj_id in (0x02, 0x03):
            candidates = candidates_by_id.get(obj_id, [])
            if candidates:
                selected[obj_id] = max(candidates, key=lambda c: c["score"])

        red_candidates = candidates_by_id.get(0x01, [])
        if red_candidates:
            green = selected.get(0x02)
            blue = selected.get(0x03)
            if green and blue:
                lo = min(green["cx"], blue["cx"]) - 70.0
                hi = max(green["cx"], blue["cx"]) + 70.0
                between_candidates = [c for c in red_candidates if lo <= c["cx"] <= hi]
                if between_candidates:
                    midpoint = (green["cx"] + blue["cx"]) / 2.0
                    red_candidates = between_candidates
                    for candidate in red_candidates:
                        candidate["score"] -= abs(candidate["cx"] - midpoint) * 2.5

            selected[0x01] = max(red_candidates, key=lambda c: c["score"])

        detected = []
        for obj_id in (0x01, 0x02, 0x03):
            candidate = selected.get(obj_id)
            if not candidate:
                continue
            detected.append((obj_id, candidate["cx"], candidate["area"], candidate["bbox"]))

        return detected

    def range_at_bearing(self, bearing):
        with self.latest_scan_lock:
            scan = self.latest_scan
            if scan is None or time.monotonic() - self.last_scan_time > self.sensor_timeout:
                return None
            if (rospy.Time.now() - scan.header.stamp).to_sec() > self.sensor_timeout:
                return None
            point = associate_lidar(scan, bearing, self.front_deg, self.angle_sign,
                                    self.camera_x, self.camera_y)
            return None if point is None else base_to_map(point, self.pose.copy())

    def open_camera(self):
        candidates = [0]
        by_id_dir = "/dev/v4l/by-id"
        if os.path.isdir(by_id_dir):
            for name in sorted(os.listdir(by_id_dir)):
                if "video-index0" in name:
                    candidates.append(os.path.join(by_id_dir, name))

        candidates.extend(f"/dev/video{i}" for i in range(1, 8))

        for candidate in candidates:
            if isinstance(candidate, str) and not os.path.exists(candidate):
                continue

            cap = cv2.VideoCapture(0) if candidate == 0 else cv2.VideoCapture(candidate, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 15)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            ok, frame = cap.read()
            if ok and frame is not None:
                rospy.loginfo("USB camera opened: %s", candidate)
                return cap, int(frame.shape[1])

            cap.release()

        return None, 640

    def camera_loop(self):
        cap, frame_width = None, 640
        while cap is None and not rospy.is_shutdown():
            cap, frame_width = self.open_camera()
            if cap is None:
                rospy.logwarn_throttle(5.0, "USB camera unavailable; retrying")
                rospy.sleep(1.0)

        while not rospy.is_shutdown():
            ok, frame = cap.read()
            if not ok:
                cap.release()
                cap, frame_width = self.open_camera()
                if cap is None:
                    rospy.logwarn_throttle(5.0, "USB camera read failed; retrying")
                    rospy.sleep(1.0)
                    self.object_counts.clear()
                    continue
                self.object_counts.clear()
                rospy.sleep(0.1)
                continue

            self.last_camera_time = time.monotonic()
            self.frame_index += 1
            if self.frame_index % 3 != 0:
                continue

            if time.monotonic() - self.last_processed_frame_time > self.sensor_timeout:
                self.object_counts.clear()
            self.last_processed_frame_time = time.monotonic()
            frame_width = frame.shape[1]
            detected = self.detect_objects(frame)
            current_detections = []
            detected_ids = {obj_id for obj_id, _, _, _ in detected}
            for obj_id in OBJECTS:
                if obj_id not in detected_ids:
                    self.object_counts[obj_id] = 0

            debug = frame.copy()
            for obj_id, center_x, area, bbox in detected:
                x, y, w, h = bbox
                info = OBJECTS[obj_id]
                cv2.rectangle(debug, (x, y), (x + w, y + h), info["bgr"], 2)
                cv2.putText(
                    debug,
                    info["name"],
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    info["bgr"],
                    2,
                    cv2.LINE_AA,
                )
                self.object_counts[obj_id] += 1
                confirmed = self.object_counts[obj_id] >= self.confirm_frames
                detection = {"id": obj_id, "name": info["name"],
                             "bbox": list(bbox), "confirmed": confirmed,
                             "position_status": "unlocated"}
                current_detections.append(detection)
                if not confirmed:
                    continue

                bearing = self.camera_yaw + camera_bearing(center_x, frame_width, self.camera_fov_rad)
                position = self.range_at_bearing(bearing)
                if position is None:
                    continue
                x, y = position
                stamp = time.time()
                detection["position_status"] = "estimated_lidar_bearing"
                with self.latest_scan_lock:
                    previous = self.object_positions.get(obj_id)
                    if previous and math.hypot(x - previous[0], y - previous[1]) < 0.4:
                        x = 0.8 * previous[0] + 0.2 * x
                        y = 0.8 * previous[1] + 0.2 * y
                    self.object_positions[obj_id] = (x, y, stamp)
                    self.object_metadata[obj_id] = {
                        "source": "lidar_bearing_association", "estimated": True,
                        "confirmed_frames": self.object_counts[obj_id],
                        "height_match_verified": False,
                    }
                rospy.loginfo_throttle(
                    2.0,
                    "Object %s estimated at x=%.2f y=%.2f",
                    OBJECTS[obj_id]["name"],
                    x,
                    y,
                )

            with self.latest_scan_lock:
                self.current_detections = current_detections
            now = time.time()
            if now - self.last_camera_debug_time > 1.0:
                self.write_image("camera_debug_latest.jpg", debug)
                self.last_camera_debug_time = now

        cap.release()

    def save_map_png(self):
        occ = np.full(self.logodds.shape, 128, dtype=np.uint8)
        occ[self.logodds < -0.35] = 245
        occ[self.logodds > 0.75] = 20
        img = cv2.cvtColor(np.flipud(occ), cv2.COLOR_GRAY2BGR)

        robot_cx, robot_cy = self.world_to_cell(self.pose[0], self.pose[1])
        robot_py = self.size - 1 - robot_cy
        if self.cell_inside(robot_cx, robot_cy):
            cv2.circle(img, (robot_cx, robot_py), 5, (0, 255, 255), -1)
            tip = (
                int(robot_cx + 18 * math.cos(self.pose[2])),
                int(robot_py - 18 * math.sin(self.pose[2])),
            )
            cv2.line(img, (robot_cx, robot_py), tip, (0, 255, 255), 2)

        for obj_id, (x, y, stamp) in self.object_positions.items():
            cx, cy = self.world_to_cell(x, y)
            py = self.size - 1 - cy
            if not self.cell_inside(cx, cy):
                continue
            info = OBJECTS[obj_id]
            cv2.circle(img, (cx, py), 7, info["bgr"], -1)
            cv2.putText(
                img,
                info["name"],
                (cx + 8, py - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                info["bgr"],
                1,
                cv2.LINE_AA,
            )

        self.write_image("map_latest.png", img)
        self.write_image("map.pgm", np.flipud(occ))
        known = np.argwhere(np.abs(self.logodds) > 0.35)
        if len(known):
            cells = [(int(cy), int(cx)) for cy, cx in known[::max(1, len(known) // 10000)]]
            cells.append((robot_cy, robot_cx))
            for obj_id, (x, y, _) in self.object_positions.items():
                cx, cy = self.world_to_cell(x, y)
                text_size, _ = cv2.getTextSize(OBJECTS[obj_id]["name"], cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
                cells.extend(((cy - 8, cx - 8), (cy + text_size[1] + 10, cx + text_size[0] + 10)))
            ys, xs = zip(*cells)
            x0, x1 = max(0, min(xs) - 25), min(self.size, max(xs) + 26)
            y0, y1 = max(0, min(ys) - 25), min(self.size, max(ys) + 26)
            preview = img[self.size - y1:self.size - y0, x0:x1]
            if preview.size:
                scale = max(1, min(4, 900 // max(preview.shape[:2])))
                self.write_image("map_preview.png", cv2.resize(preview, None, fx=scale, fy=scale,
                                                              interpolation=cv2.INTER_NEAREST))
        yaml_text = (f"image: map.pgm\nresolution: {self.resolution}\n"
                     f"origin: [{self.origin_x}, {self.origin_y}, 0.0]\n"
                     "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
        self.write_text("map.yaml", yaml_text)
        self.write_text("objects.json", json.dumps({
            "frame_id": self.map_frame, "units": "m", "saved_at": time.time(),
            "calibration": {"front_deg": self.front_deg, "angle_sign": self.angle_sign,
                            "camera_fov_deg": math.degrees(self.camera_fov_rad),
                            "camera_x_m": self.camera_x, "camera_y_m": self.camera_y,
                            "camera_yaw_deg": math.degrees(self.camera_yaw)},
            "objects": [dict(id=obj_id, name=OBJECTS[obj_id]["name"], x=float(x), y=float(y),
                             last_seen=stamp, **self.object_metadata.get(obj_id, {}))
                        for obj_id, (x, y, stamp) in self.object_positions.items()],
            "current_detections": self.current_detections,
        }, indent=2, allow_nan=False))
        self.write_text("status.json", json.dumps({
            "saved_at": time.time(), "scan_count": self.scan_count,
            "camera_frames": self.frame_index,
            "camera_ok": self.enable_camera and time.monotonic() - self.last_camera_time < 2.0,
            "pose": [float(value) for value in self.pose],
            "known_cells": int(len(known)), "mapped_objects": len(self.object_positions),
        }, indent=2))

    def write_image(self, filename, image):
        stem, extension = os.path.splitext(filename)
        temporary = os.path.join(self.output_dir, stem + ".tmp" + extension)
        if not cv2.imwrite(temporary, image):
            raise OSError("Cannot write " + temporary)
        os.replace(temporary, os.path.join(self.output_dir, filename))

    def write_text(self, filename, content):
        destination = os.path.join(self.output_dir, filename)
        with open(destination + ".tmp", "w", encoding="utf-8") as handle:
            handle.write(content)
        os.replace(destination + ".tmp", destination)


def main():
    rospy.init_node("simple_slam_object_mapper")
    mapper = SimpleSlamObjectMapper()
    rospy.loginfo("Simple SLAM/object mapper running. Publishing /map and /object_markers.")
    rospy.spin()


if __name__ == "__main__":
    main()
