#!/usr/bin/env python3
import math
import subprocess
import time

import rospy
from sensor_msgs.msg import LaserScan

DIRECT_CAN_ID = 0x210

RUN_SECONDS = 180.0
CONTROL_HZ = 10.0
COMMAND_DURATION_MS = 500
COMMAND_REFRESH_SECONDS = 0.35

SPEED_FAST = 50
SPEED_SLOW = 50
BACKWARD_SPEED = 40
TURN_SPEED = 45

MIN_RANGE_M = 0.05
MAX_RANGE_M = 5.0
FRONT_STOP_DISTANCE = 0.25
FRONT_RELEASE_DISTANCE = 0.35
SIDE_AVOID_DISTANCE = 0.055
SIDE_COMPARE_HYSTERESIS = 0.02
LIDAR_TIMEOUT = 1.0

BRAKE_TIME = 0.20
BACKWARD_TIME = 0.50
TURN_TIME = 0.70
WAIT_CLEAR_TIME = 0.20
STUCK_DETECT_SECONDS = 5.0
STUCK_DISTANCE_EPSILON = 0.02
RECOVERY_BACKWARD_TIME = 5.0
RECOVERY_REAR_CLEARANCE = 0.30
RECOVERY_COOLDOWN = 10.0

CENTER_WIDTH_DEG = 15.0
SIDE_OFFSET_DEG = 80.0
SIDE_WIDTH_DEG = 15.0

SENSOR_FORWARD_SIGN = 1
LEFT_WHEEL_SCALE = 0.98
RIGHT_WHEEL_SCALE = 1.00
STM_LEFT_RIGHT_SWAPPED = False
DEFAULT_TURN = "RIGHT"

STATE_IDLE = "IDLE"
STATE_BRAKE = "BRAKE"
STATE_BACKWARD = "BACKWARD"
STATE_TURN_LEFT = "TURN_LEFT"
STATE_TURN_RIGHT = "TURN_RIGHT"
STATE_WAIT_CLEAR = "WAIT_CLEAR"
STATE_SENSOR_ERROR = "SENSOR_ERROR"
STATE_STUCK_RECOVERY = "STUCK_RECOVERY"


class RosScanCanDrive:
    def __init__(self):
        self.front_deg = float(rospy.get_param("~front_deg", -97.0))
        self.latest_scan = None
        self.last_scan_time = 0.0
        self.last_frame = None
        self.last_send_time = 0.0
        self.state = STATE_IDLE
        self.state_enter_time = time.monotonic()
        self.turn_direction = DEFAULT_TURN
        self.last_motion_command = "STOP"
        self.stuck_since = None
        self.stuck_reference_center = None
        self.avoidance_stuck_since = None
        self.avoidance_stuck_reference_center = None
        self.recovery_cooldown_until = 0.0
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

    def scan_callback(self, scan):
        self.latest_scan = scan
        self.last_scan_time = time.monotonic()

    def wait_for_scan(self, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.latest_scan is not None:
                return True
            rospy.sleep(0.05)
        return False

    def send_wheel(self, left, right, duration_ms, force=False):
        left = max(-100, min(100, int(round(left))))
        right = max(-100, min(100, int(round(right))))
        left = int(round(left * LEFT_WHEEL_SCALE))
        right = int(round(right * RIGHT_WHEEL_SCALE))
        if STM_LEFT_RIGHT_SWAPPED:
            left, right = right, left

        duration_ms = max(0, min(65535, int(duration_ms)))
        frame = (
            f"{DIRECT_CAN_ID:03X}#"
            f"{left & 0xFF:02X}"
            f"{right & 0xFF:02X}"
            f"{duration_ms & 0xFF:02X}"
            f"{(duration_ms >> 8) & 0xFF:02X}"
        )
        now = time.monotonic()
        if not force and frame == self.last_frame and now - self.last_send_time < COMMAND_REFRESH_SECONDS:
            return
        subprocess.run(["cansend", "can0", frame], check=False)
        self.last_frame = frame
        self.last_send_time = now
        rospy.loginfo("CAN %s", frame)

    def send_drive_command(self, command, force=False):
        self.last_motion_command = command
        if command == "FORWARD":
            self.send_wheel(SENSOR_FORWARD_SIGN * SPEED_FAST, SENSOR_FORWARD_SIGN * SPEED_FAST, COMMAND_DURATION_MS, force)
        elif command == "SLOW_FORWARD":
            self.send_wheel(SENSOR_FORWARD_SIGN * SPEED_SLOW, SENSOR_FORWARD_SIGN * SPEED_SLOW, COMMAND_DURATION_MS, force)
        elif command == "BACKWARD":
            self.send_wheel(-SENSOR_FORWARD_SIGN * BACKWARD_SPEED, -SENSOR_FORWARD_SIGN * BACKWARD_SPEED, COMMAND_DURATION_MS, force)
        elif command == "LEFT":
            self.send_wheel(SENSOR_FORWARD_SIGN * TURN_SPEED, -SENSOR_FORWARD_SIGN * TURN_SPEED, COMMAND_DURATION_MS, force)
        elif command == "RIGHT":
            self.send_wheel(-SENSOR_FORWARD_SIGN * TURN_SPEED, SENSOR_FORWARD_SIGN * TURN_SPEED, COMMAND_DURATION_MS, force)
        else:
            self.send_wheel(0, 0, 0, True)

    def stop(self):
        self.send_drive_command("STOP", force=True)

    def set_state(self, new_state):
        if self.state != new_state:
            rospy.loginfo("STATE %s -> %s", self.state, new_state)
            self.state = new_state
            self.state_enter_time = time.monotonic()

    def state_elapsed(self):
        return time.monotonic() - self.state_enter_time

    def reset_stuck_detector(self):
        self.stuck_since = None
        self.stuck_reference_center = None

    def reset_avoidance_stuck_detector(self):
        self.avoidance_stuck_since = None
        self.avoidance_stuck_reference_center = None

    def detect_stuck_forward(self, center):
        now = time.monotonic()
        if (self.state != STATE_IDLE
                or self.last_motion_command not in ("FORWARD", "SLOW_FORWARD")
                or not math.isfinite(center)
                or now < self.recovery_cooldown_until):
            self.reset_stuck_detector()
            return False

        if self.stuck_since is None or self.stuck_reference_center is None:
            self.stuck_since = now
            self.stuck_reference_center = center
            return False

        if abs(center - self.stuck_reference_center) > STUCK_DISTANCE_EPSILON:
            self.stuck_since = now
            self.stuck_reference_center = center
            return False

        return now - self.stuck_since >= STUCK_DETECT_SECONDS

    def detect_stuck_avoidance(self, center):
        now = time.monotonic()
        avoidance_states = (
            STATE_BRAKE,
            STATE_BACKWARD,
            STATE_TURN_LEFT,
            STATE_TURN_RIGHT,
            STATE_WAIT_CLEAR,
        )
        if (self.state not in avoidance_states
                or not math.isfinite(center)
                or now < self.recovery_cooldown_until):
            self.reset_avoidance_stuck_detector()
            return False

        if (self.avoidance_stuck_since is None
                or self.avoidance_stuck_reference_center is None):
            self.avoidance_stuck_since = now
            self.avoidance_stuck_reference_center = center
            return False

        if abs(center - self.avoidance_stuck_reference_center) > STUCK_DISTANCE_EPSILON:
            self.avoidance_stuck_since = now
            self.avoidance_stuck_reference_center = center
            return False

        return now - self.avoidance_stuck_since >= STUCK_DETECT_SECONDS

    def sector_min_distance(self, center_deg, width_deg):
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
        return min(values)

    def read_distances(self):
        left = self.sector_min_distance(self.front_deg - SIDE_OFFSET_DEG, SIDE_WIDTH_DEG)
        center = self.sector_min_distance(self.front_deg, CENTER_WIDTH_DEG)
        right = self.sector_min_distance(self.front_deg + SIDE_OFFSET_DEG, SIDE_WIDTH_DEG)
        return left, center, right

    def choose_turn(self, left_distance, right_distance):
        left_seen = math.isfinite(left_distance)
        right_seen = math.isfinite(right_distance)
        if left_seen and right_seen:
            if left_distance - right_distance > SIDE_COMPARE_HYSTERESIS:
                return "LEFT"
            if right_distance - left_distance > SIDE_COMPARE_HYSTERESIS:
                return "RIGHT"
            return DEFAULT_TURN
        if left_seen and not right_seen:
            return "RIGHT"
        if right_seen and not left_seen:
            return "LEFT"
        return DEFAULT_TURN

    def update(self):
        now = time.monotonic()
        if self.latest_scan is None or now - self.last_scan_time > LIDAR_TIMEOUT:
            self.set_state(STATE_SENSOR_ERROR)
            self.stop()
            return

        left, center, right = self.read_distances()
        # An empty front sector means measurement failure, not unlimited clearance.
        if not math.isfinite(center):
            self.set_state(STATE_SENSOR_ERROR)
            self.stop()
            rospy.logwarn("No valid LiDAR measurement in the front sector")
            return

        front_blocked = center <= FRONT_STOP_DISTANCE
        front_clear = center >= FRONT_RELEASE_DISTANCE
        left_blocked = left <= SIDE_AVOID_DISTANCE
        right_blocked = right <= SIDE_AVOID_DISTANCE
        all_blocked = front_blocked and left_blocked and right_blocked
        rear = self.sector_min_distance(self.front_deg + 180.0, CENTER_WIDTH_DEG)

        rospy.loginfo("STATE=%s L=%.2f C=%.2f R=%.2f", self.state, left, center, right)

        if self.state == STATE_SENSOR_ERROR:
            self.set_state(STATE_IDLE)
            self.stop()
            return

        if self.state == STATE_IDLE and self.detect_stuck_forward(center):
            rospy.logwarn("Forward motion appears stuck for %.1fs; entering recovery", STUCK_DETECT_SECONDS)
            self.stop()
            self.set_state(STATE_STUCK_RECOVERY)
            self.reset_stuck_detector()
            return

        if self.detect_stuck_avoidance(center):
            rospy.logwarn("Avoidance appears stuck for %.1fs; entering recovery", STUCK_DETECT_SECONDS)
            self.stop()
            self.set_state(STATE_STUCK_RECOVERY)
            self.reset_avoidance_stuck_detector()
            return

        if self.state == STATE_IDLE:
            if front_blocked:
                self.turn_direction = self.choose_turn(left, right)
                self.stop()
                self.set_state(STATE_BRAKE)
            elif center < FRONT_RELEASE_DISTANCE:
                self.send_drive_command("SLOW_FORWARD")
            else:
                self.send_drive_command("FORWARD")
            return

        if self.state == STATE_BRAKE:
            self.stop()
            if self.state_elapsed() < BRAKE_TIME:
                return
            if all_blocked:
                self.set_state(STATE_BACKWARD)
            else:
                self.set_state(STATE_TURN_LEFT if self.turn_direction == "LEFT" else STATE_TURN_RIGHT)
            return

        if self.state == STATE_BACKWARD:
            self.send_drive_command("BACKWARD")
            if self.state_elapsed() >= BACKWARD_TIME:
                self.turn_direction = self.choose_turn(left, right)
                self.stop()
                self.set_state(STATE_TURN_LEFT if self.turn_direction == "LEFT" else STATE_TURN_RIGHT)
            return

        if self.state == STATE_TURN_LEFT:
            self.send_drive_command("LEFT")
            if self.state_elapsed() >= TURN_TIME:
                self.stop()
                self.set_state(STATE_WAIT_CLEAR)
            return

        if self.state == STATE_TURN_RIGHT:
            self.send_drive_command("RIGHT")
            if self.state_elapsed() >= TURN_TIME:
                self.stop()
                self.set_state(STATE_WAIT_CLEAR)
            return

        if self.state == STATE_WAIT_CLEAR:
            self.stop()
            if self.state_elapsed() < WAIT_CLEAR_TIME:
                return
            if front_clear:
                self.set_state(STATE_IDLE)
            else:
                self.turn_direction = self.choose_turn(left, right)
                self.set_state(STATE_BACKWARD if all_blocked else STATE_BRAKE)

        if self.state == STATE_STUCK_RECOVERY:
            if self.state_elapsed() < BRAKE_TIME:
                return

            if not math.isfinite(rear) or rear < RECOVERY_REAR_CLEARANCE:
                rospy.logwarn("Rear clearance is unsafe or unknown (%.2fm); skipping recovery reverse", rear)
                self.turn_direction = self.choose_turn(left, right)
                self.recovery_cooldown_until = time.monotonic() + RECOVERY_COOLDOWN
                self.set_state(STATE_TURN_LEFT if self.turn_direction == "LEFT" else STATE_TURN_RIGHT)
                return

            self.send_drive_command("BACKWARD")
            if self.state_elapsed() >= BRAKE_TIME + RECOVERY_BACKWARD_TIME:
                self.stop()
                self.turn_direction = self.choose_turn(left, right)
                self.recovery_cooldown_until = time.monotonic() + RECOVERY_COOLDOWN
                self.set_state(STATE_TURN_LEFT if self.turn_direction == "LEFT" else STATE_TURN_RIGHT)
            return

    def run(self):
        if not self.wait_for_scan():
            rospy.logerr("No /scan received")
            self.stop()
            return 1
        rate = rospy.Rate(CONTROL_HZ)
        end_time = time.monotonic() + RUN_SECONDS
        try:
            while not rospy.is_shutdown() and time.monotonic() < end_time:
                self.update()
                rate.sleep()
        finally:
            self.stop()
        return 0


def main():
    rospy.init_node("ros_scan_can_drive")
    node = RosScanCanDrive()
    return node.run()


if __name__ == "__main__":
    raise SystemExit(main())
