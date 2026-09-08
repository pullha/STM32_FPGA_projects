#!/usr/bin/env python3
import math
import statistics
import subprocess
import time

import rospy
from sensor_msgs.msg import LaserScan

DIRECT_CAN_ID = 0x210
RUN_SECONDS = 60.0
FORWARD_MS = 320
SPEED_FAST = 50
SPEED_SLOW = 40
MIN_RANGE_M = 0.30
MAX_RANGE_M = 5.0
FRONT_STOP_M = 0.38
FRONT_CLEAR_M = 0.62
SIDE_MIN_CLEAR_M = 0.11
CENTER_STEER_GAIN = 14.0
ONE_WALL_STEER = 5
MAX_STEER = 8
SIDE_LOST_DISTANCE_M = 0.55
PIVOT_SPEED = 55
PIVOT_MS = 360
PIVOT_SETTLE_SECONDS = 0.25
SENSOR_FORWARD_SIGN = 1
LEFT_WHEEL_SCALE = 0.96
RIGHT_WHEEL_SCALE = 1.00


class RosScanCanDrive:
    def __init__(self):
        self.front_deg = float(rospy.get_param("~front_deg", 162.1))
        self.latest_scan = None
        self.last_frame = None
        self.last_send_time = 0.0
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

    def scan_callback(self, scan):
        self.latest_scan = scan

    def wait_for_scan(self, timeout=8.0):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.latest_scan is not None:
                return True
            rospy.sleep(0.05)
        return False

    def send_wheel(self, left, right, duration_ms, force=False):
        left = max(-100, min(100, int(left)))
        right = max(-100, min(100, int(right)))
        duration_ms = max(0, min(65535, int(duration_ms)))
        frame = (
            f"{DIRECT_CAN_ID:03X}#"
            f"{left & 0xFF:02X}"
            f"{right & 0xFF:02X}"
            f"{duration_ms & 0xFF:02X}"
            f"{(duration_ms >> 8) & 0xFF:02X}"
        )
        now = time.time()
        if not force and frame == self.last_frame and now - self.last_send_time < 0.18:
            return
        subprocess.run(["cansend", "can0", frame], check=False)
        self.last_frame = frame
        self.last_send_time = now
        rospy.loginfo("CAN %s", frame)

    def stop(self):
        self.send_wheel(0, 0, 0, force=True)

    def median_in_sector(self, center_deg, width_deg):
        scan = self.latest_scan
        if scan is None:
            return float("inf")
        values = []
        center = math.radians(center_deg)
        half = math.radians(width_deg / 2.0)
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < MIN_RANGE_M or distance > MAX_RANGE_M:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            diff = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(diff) <= half:
                values.append(distance)
        if not values:
            return float("inf")
        return statistics.median(values)

    def drive_forward(self, speed, steer_delta):
        steer = max(-MAX_STEER, min(MAX_STEER, int(steer_delta)))
        left = SENSOR_FORWARD_SIGN * speed
        right = SENSOR_FORWARD_SIGN * speed
        if steer > 0:
            left = SENSOR_FORWARD_SIGN * (speed - steer)
        elif steer < 0:
            right = SENSOR_FORWARD_SIGN * (speed + steer)
        left = int(round(left * LEFT_WHEEL_SCALE))
        right = int(round(right * RIGHT_WHEEL_SCALE))
        self.send_wheel(left, right, FORWARD_MS)
        rospy.sleep(FORWARD_MS / 1000.0)

    def center_steer(self, left, right):
        left_seen = math.isfinite(left)
        right_seen = math.isfinite(right)

        if left_seen and left < SIDE_MIN_CLEAR_M:
            return -MAX_STEER

        if right_seen and right < SIDE_MIN_CLEAR_M:
            return MAX_STEER

        if left_seen and right_seen:
            return (left - right) * CENTER_STEER_GAIN

        if left_seen and left < SIDE_LOST_DISTANCE_M:
            return -ONE_WALL_STEER

        if right_seen and right < SIDE_LOST_DISTANCE_M:
            return ONE_WALL_STEER

        return 0

    def avoid_front_wall(self, left, right):
        self.stop()
        rospy.sleep(0.08)

        if left >= right:
            self.send_wheel(SENSOR_FORWARD_SIGN * PIVOT_SPEED, -SENSOR_FORWARD_SIGN * PIVOT_SPEED, PIVOT_MS, force=True)
            rospy.loginfo("AVOID left")
        else:
            self.send_wheel(-SENSOR_FORWARD_SIGN * PIVOT_SPEED, SENSOR_FORWARD_SIGN * PIVOT_SPEED, PIVOT_MS, force=True)
            rospy.loginfo("AVOID right")

        rospy.sleep(PIVOT_MS / 1000.0 + PIVOT_SETTLE_SECONDS)
        self.stop()

    def run(self):
        if not self.wait_for_scan():
            rospy.logerr("No /scan received")
            return 1
        try:
            end_time = time.time() + RUN_SECONDS

            while not rospy.is_shutdown() and time.time() < end_time:
                front = self.median_in_sector(self.front_deg, 35)
                left = self.median_in_sector(self.front_deg + 75.0, 30)
                right = self.median_in_sector(self.front_deg - 75.0, 30)
                steer = self.center_steer(left, right)
                rospy.loginfo(
                    "DIST front=%.2f left=%.2f right=%.2f steer=%.1f",
                    front,
                    left,
                    right,
                    steer,
                )

                if front < FRONT_STOP_M:
                    self.avoid_front_wall(left, right)
                elif front < FRONT_CLEAR_M:
                    self.drive_forward(SPEED_SLOW, steer)
                else:
                    self.drive_forward(SPEED_FAST, steer)
        finally:
            self.stop()
        return 0


def main():
    rospy.init_node("ros_scan_can_drive")
    node = RosScanCanDrive()
    return node.run()


if __name__ == "__main__":
    raise SystemExit(main())
