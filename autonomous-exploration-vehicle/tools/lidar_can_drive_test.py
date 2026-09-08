#!/usr/bin/env python3
import math
import json
import statistics
import subprocess
import sys
import time

sys.path.insert(0, "/home/pi/YDLidar-SDK/build/python")
import ydlidar

DIRECT_CAN_ID = 0x210

SPEED_PERCENT = 50
SPEED_SLOW = 40
FORWARD_MS = 320
TURN_SPEED = 55
TURN_MS = 360
RUN_SECONDS = 30.0
FRONT_STOP_M = 0.38
FRONT_CLEAR_M = 0.62
FRONT_CONFIG_PATH = "/home/pi/lidar_front_config.json"
SCAN_SECONDS = 5.0
RESCAN_INTERVAL_SECONDS = 3.0
RESCAN_SECONDS = 1.0
SECTOR_STEP_DEG = 15
SECTOR_WIDTH_DEG = 30
MIN_DRIVE_RANGE_M = 0.30
MAX_DRIVE_RANGE_M = 5.0
OPEN_DIRECTION_TURN_MS_PER_DEG = 7.0
CENTER_STEER_GAIN = 14.0
ONE_WALL_STEER = 5
MAX_STEER = 8
SIDE_LOST_DISTANCE_M = 0.55
SIDE_MIN_CLEAR_M = 0.11
SENSOR_FORWARD_SIGN = 1
LEFT_WHEEL_SCALE = 0.96
RIGHT_WHEEL_SCALE = 1.00


def load_front_deg():
    try:
        with open(FRONT_CONFIG_PATH, "r", encoding="utf-8") as f:
            config = json.load(f)
        return float(config.get("front_deg", 0.0))
    except (OSError, ValueError, TypeError):
        return 0.0


last_frame = None
last_send_time = 0.0


def signed_byte(value):
    return value & 0xFF


def send_wheel_cmd(left_percent, right_percent, duration_ms=0, force=False):
    global last_frame, last_send_time

    left_percent = max(-100, min(100, int(left_percent)))
    right_percent = max(-100, min(100, int(right_percent)))
    duration_ms = max(0, min(65535, int(duration_ms)))
    frame = (
        f"{DIRECT_CAN_ID:03X}#"
        f"{signed_byte(left_percent):02X}"
        f"{signed_byte(right_percent):02X}"
        f"{duration_ms & 0xFF:02X}"
        f"{(duration_ms >> 8) & 0xFF:02X}"
    )

    now = time.time()
    if not force and frame == last_frame and now - last_send_time < 0.18:
        return

    subprocess.run(["cansend", "can0", frame], check=False)
    last_frame = frame
    last_send_time = now
    print("CAN", frame, flush=True)


def send_stop(force=False):
    send_wheel_cmd(0, 0, 0, force=force)


def send_forward_steer(base_speed, steer_delta, duration_ms):
    # The lidar/camera deck is the robot front. Change SENSOR_FORWARD_SIGN if
    # the chassis moves toward the motor side during a forward command.
    steer = max(-MAX_STEER, min(MAX_STEER, int(steer_delta)))
    left = SENSOR_FORWARD_SIGN * base_speed
    right = SENSOR_FORWARD_SIGN * base_speed

    if steer > 0:
        left = SENSOR_FORWARD_SIGN * (base_speed - steer)
    elif steer < 0:
        right = SENSOR_FORWARD_SIGN * (base_speed + steer)

    left = int(round(left * LEFT_WHEEL_SCALE))
    right = int(round(right * RIGHT_WHEEL_SCALE))
    send_wheel_cmd(left, right, duration_ms)


def center_steer(left, right):
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


def avoid_front_wall(left, right):
    send_stop(force=True)
    time.sleep(0.08)

    if left >= right:
        send_wheel_cmd(SENSOR_FORWARD_SIGN * TURN_SPEED, -SENSOR_FORWARD_SIGN * TURN_SPEED, TURN_MS, force=True)
        print("AVOID left", flush=True)
    else:
        send_wheel_cmd(-SENSOR_FORWARD_SIGN * TURN_SPEED, SENSOR_FORWARD_SIGN * TURN_SPEED, TURN_MS, force=True)
        print("AVOID right", flush=True)

    time.sleep(TURN_MS / 1000.0)
    send_stop(force=True)


def median_in_sector(scan, center_deg, width_deg):
    values = []
    half = math.radians(width_deg / 2.0)
    center = math.radians(center_deg)

    for point in scan.points:
        distance = point.range
        if not math.isfinite(distance) or distance < MIN_DRIVE_RANGE_M or distance > MAX_DRIVE_RANGE_M:
            continue

        diff = math.atan2(
            math.sin(point.angle - center),
            math.cos(point.angle - center),
        )
        if abs(diff) <= half:
            values.append(distance)

    if not values:
        return float("inf")
    return statistics.median(values)


def angle_diff_deg(target_deg, base_deg):
    return math.degrees(
        math.atan2(
            math.sin(math.radians(target_deg - base_deg)),
            math.cos(math.radians(target_deg - base_deg)),
        )
    )


def scan_open_direction(laser, scan, front_deg, scan_seconds=SCAN_SECONDS):
    sector_samples = {
        angle: []
        for angle in range(-180, 180, SECTOR_STEP_DEG)
    }
    deadline = time.time() + scan_seconds

    print(f"OPEN_SCAN_START seconds={scan_seconds:.1f}", flush=True)
    while time.time() < deadline:
        if not laser.doProcessSimple(scan):
            time.sleep(0.05)
            continue

        for angle in sector_samples:
            distance = median_in_sector(scan, angle, SECTOR_WIDTH_DEG)
            if math.isfinite(distance):
                sector_samples[angle].append(distance)

    best_angle = front_deg
    best_distance = -1.0
    for angle, samples in sector_samples.items():
        if len(samples) < 3:
            continue

        # Use a conservative median so one noisy long ray does not win.
        distance = statistics.median(samples)
        if distance > best_distance:
            best_distance = distance
            best_angle = float(angle)

    turn_delta = angle_diff_deg(best_angle, front_deg)
    print(
        f"OPEN_SCAN_OK best_angle={best_angle:.1f}deg "
        f"best_distance={best_distance:.2f}m turn_delta={turn_delta:.1f}deg",
        flush=True,
    )
    return best_angle, best_distance, turn_delta


def turn_toward_open_direction(turn_delta):
    if abs(turn_delta) < 18.0:
        print("OPEN_TURN_SKIP", flush=True)
        return

    turn_ms = int(min(1400, max(220, abs(turn_delta) * OPEN_DIRECTION_TURN_MS_PER_DEG)))
    if turn_delta > 0:
        print(f"OPEN_TURN_LEFT ms={turn_ms}", flush=True)
        send_wheel_cmd(SENSOR_FORWARD_SIGN * TURN_SPEED, -SENSOR_FORWARD_SIGN * TURN_SPEED, turn_ms, force=True)
    else:
        print(f"OPEN_TURN_RIGHT ms={turn_ms}", flush=True)
        send_wheel_cmd(-SENSOR_FORWARD_SIGN * TURN_SPEED, SENSOR_FORWARD_SIGN * TURN_SPEED, turn_ms, force=True)

    time.sleep(turn_ms / 1000.0 + 0.15)
    send_stop(force=True)


def main():
    front_deg = load_front_deg()
    left_deg = front_deg + 70.0
    right_deg = front_deg - 70.0

    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/ttyUSB0")
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 5)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 10.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)

    if not laser.initialize():
        print("LIDAR_INIT_FAILED")
        return 1

    if not laser.turnOn():
        print("LIDAR_TURN_ON_FAILED")
        laser.disconnecting()
        return 1

    scan = ydlidar.LaserScan()
    deadline = time.time() + RUN_SECONDS
    print("LIDAR_CAN_DRIVE_START", flush=True)
    print(
        f"FRONT_DEG={front_deg:.1f} LEFT_DEG={left_deg:.1f} RIGHT_DEG={right_deg:.1f}",
        flush=True,
    )

    try:
        while time.time() < deadline:
            if not laser.doProcessSimple(scan):
                send_stop()
                time.sleep(0.05)
                continue

            front = median_in_sector(scan, front_deg, 35)
            left = median_in_sector(scan, left_deg, 30)
            right = median_in_sector(scan, right_deg, 30)
            steer = center_steer(left, right)
            print(
                f"DIST front={front:.2f}m left={left:.2f}m right={right:.2f}m steer={steer:.1f}",
                flush=True,
            )

            if front < FRONT_STOP_M:
                avoid_front_wall(left, right)
            elif front < FRONT_CLEAR_M:
                send_forward_steer(SPEED_SLOW, steer, FORWARD_MS)
                time.sleep(FORWARD_MS / 1000.0)
            else:
                send_forward_steer(SPEED_PERCENT, steer, FORWARD_MS)
                time.sleep(FORWARD_MS / 1000.0)
    finally:
        send_stop()
        laser.turnOff()
        laser.disconnecting()
        print("LIDAR_CAN_DRIVE_STOP", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
