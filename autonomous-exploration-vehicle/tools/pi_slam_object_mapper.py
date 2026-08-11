#!/usr/bin/env python3
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
        self.laser_frame = rospy.get_param("~laser_frame", "laser")
        self.camera_fov_rad = math.radians(float(rospy.get_param("~camera_fov_deg", 62.0)))
        self.enable_camera = bool(rospy.get_param("~enable_camera", True))
        self.output_dir = rospy.get_param("~output_dir", "/home/pi/slam_outputs")
        os.makedirs(self.output_dir, exist_ok=True)

        self.logodds = np.zeros((self.size, self.size), dtype=np.float32)
        self.pose = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.latest_scan = None
        self.latest_scan_lock = threading.Lock()
        self.object_positions = {}
        self.object_counts = defaultdict(int)
        self.last_save_time = 0.0
        self.last_camera_debug_time = 0.0
        self.frame_index = 0

        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/object_markers", MarkerArray, queue_size=1, latch=True)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

        self.camera_thread = None
        if self.enable_camera:
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()

    def world_to_cell(self, x, y):
        cx = int((x - self.origin_x) / self.resolution)
        cy = int((y - self.origin_y) / self.resolution)
        return cx, cy

    def cell_inside(self, cx, cy):
        return 0 <= cx < self.size and 0 <= cy < self.size

    def scan_points(self, scan, step=5):
        points = []
        for i in range(0, len(scan.ranges), step):
            r = scan.ranges[i]
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            points.append((float(r), float(angle)))
        return points

    def scan_match(self, points):
        occupied_known = np.count_nonzero(self.logodds > 0.7)
        if occupied_known < 80 or not points:
            return self.pose

        best_pose = self.pose.copy()
        best_score = -1e9
        dx_candidates = (-0.18, -0.09, 0.0, 0.09, 0.18)
        dy_candidates = (-0.12, -0.06, 0.0, 0.06, 0.12)
        dt_candidates = (-0.12, -0.06, 0.0, 0.06, 0.12)

        for dx in dx_candidates:
            for dy in dy_candidates:
                for dt in dt_candidates:
                    x = self.pose[0] + dx
                    y = self.pose[1] + dy
                    theta = self.pose[2] + dt
                    cos_t = math.cos(theta)
                    sin_t = math.sin(theta)
                    score = 0.0

                    for r, a in points[::2]:
                        lx = r * math.cos(a)
                        ly = r * math.sin(a)
                        wx = x + cos_t * lx - sin_t * ly
                        wy = y + sin_t * lx + cos_t * ly
                        cx, cy = self.world_to_cell(wx, wy)
                        if not self.cell_inside(cx, cy):
                            continue
                        score += self.logodds[cy, cx]

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
            for cx, cy in ray[:-1:2]:
                self.logodds[cy, cx] = clamp(self.logodds[cy, cx] - 0.18, -3.0, 3.5)
            self.logodds[end_cy, end_cx] = clamp(self.logodds[end_cy, end_cx] + 0.85, -3.0, 3.5)

    def scan_callback(self, scan):
        with self.latest_scan_lock:
            self.latest_scan = scan

        points = self.scan_points(scan)
        if points:
            self.pose = self.scan_match(points)
            self.integrate_scan(points)

        self.publish_tf()
        now = time.time()
        if now - self.last_save_time > 1.0:
            self.publish_map()
            self.publish_markers()
            self.save_map_png()
            self.last_save_time = now

    def publish_tf(self):
        stamp = rospy.Time.now()
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
        return np.flipud(occ).reshape(-1).tolist()

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
        robot.pose.orientation.w = 1.0
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
            label.text = info["name"]
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
                    shape_ok = 4 <= len(approx) <= 7
                elif obj_id == 0x03:
                    shape_ok = 3 <= len(approx) <= 6

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
        if scan is None:
            return None

        center = int(round((bearing - scan.angle_min) / scan.angle_increment))
        window = max(2, int(math.radians(12.0) / abs(scan.angle_increment)))
        values = []
        for i in range(center - window, center + window + 1):
            if 0 <= i < len(scan.ranges):
                r = scan.ranges[i]
                if math.isfinite(r) and scan.range_min <= r <= min(scan.range_max, 5.0):
                    values.append(r)
        if not values:
            return None
        return float(np.median(values))

    def estimate_distance_from_bbox(self, bbox):
        _, _, w, h = bbox
        apparent_size = max(w, h, 1)
        return clamp(180.0 / float(apparent_size), 0.45, 3.5)

    def open_camera(self):
        candidates = []
        by_id_dir = "/dev/v4l/by-id"
        if os.path.isdir(by_id_dir):
            for name in sorted(os.listdir(by_id_dir)):
                if "video-index0" in name:
                    candidates.append(os.path.join(by_id_dir, name))

        candidates.extend(f"/dev/video{i}" for i in range(0, 8))

        for candidate in candidates:
            if not os.path.exists(candidate):
                continue

            cap = cv2.VideoCapture(candidate, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 15)

            ok, frame = cap.read()
            if ok and frame is not None:
                rospy.loginfo("USB camera opened: %s", candidate)
                return cap, int(frame.shape[1])

            cap.release()

        return None, 640

    def camera_loop(self):
        cap, frame_width = self.open_camera()
        if cap is None:
            rospy.logwarn("USB camera open failed. Object markers will be disabled.")
            return

        while not rospy.is_shutdown():
            ok, frame = cap.read()
            if not ok:
                rospy.sleep(0.1)
                continue

            self.frame_index += 1
            if self.frame_index % 3 != 0:
                continue

            detected = self.detect_objects(frame)
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
                if self.object_counts[obj_id] < 5:
                    continue

                norm = (center_x - frame_width / 2.0) / (frame_width / 2.0)
                bearing = -norm * (self.camera_fov_rad / 2.0)
                distance = self.range_at_bearing(bearing)
                if distance is None:
                    distance = self.estimate_distance_from_bbox(bbox)

                world_angle = float(self.pose[2]) + bearing
                x = float(self.pose[0]) + distance * math.cos(world_angle)
                y = float(self.pose[1]) + distance * math.sin(world_angle)
                self.object_positions[obj_id] = (x, y, time.time())
                rospy.loginfo_throttle(
                    2.0,
                    "Object %s at x=%.2f y=%.2f range=%.2f",
                    OBJECTS[obj_id]["name"],
                    x,
                    y,
                    distance,
                )

            now = time.time()
            if now - self.last_camera_debug_time > 1.0:
                cv2.imwrite(os.path.join(self.output_dir, "camera_debug_latest.jpg"), debug)
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

        latest = os.path.join(self.output_dir, "map_latest.png")
        cv2.imwrite(latest, img)


def main():
    rospy.init_node("simple_slam_object_mapper")
    mapper = SimpleSlamObjectMapper()
    rospy.loginfo("Simple SLAM/object mapper running. Publishing /map and /object_markers.")
    rospy.spin()


if __name__ == "__main__":
    main()
