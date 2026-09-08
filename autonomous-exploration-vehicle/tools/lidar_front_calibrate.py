#!/usr/bin/env python3
import json
import math
import statistics
import sys
import time

sys.path.insert(0, "/home/pi/YDLidar-SDK/build/python")
import ydlidar

OUTPUT_PATH = "/home/pi/lidar_front_config.json"


def angle_mean_deg(angles_deg):
    sin_sum = sum(math.sin(math.radians(a)) for a in angles_deg)
    cos_sum = sum(math.cos(math.radians(a)) for a in angles_deg)
    return math.degrees(math.atan2(sin_sum, cos_sum))


def closest_angle(scan):
    best_angle = None
    best_distance = float("inf")
    for point in scan.points:
        distance = point.range
        if not math.isfinite(distance) or distance < 0.18 or distance > 1.2:
            continue
        if distance < best_distance:
            best_distance = distance
            best_angle = math.degrees(point.angle)
    return best_angle, best_distance


def main():
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

    print("FRONT_CALIBRATION_START")
    print("Hold your hand at the robot's physical front, about 40-60 cm away.")
    scan = ydlidar.LaserScan()
    angles = []
    distances = []

    try:
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if not laser.doProcessSimple(scan):
                time.sleep(0.05)
                continue

            angle, distance = closest_angle(scan)
            if angle is None:
                print("sample: none")
                time.sleep(0.1)
                continue

            angles.append(angle)
            distances.append(distance)
            print(f"sample: angle={angle:.1f}deg distance={distance:.2f}m", flush=True)
            time.sleep(0.15)
    finally:
        laser.turnOff()
        laser.disconnecting()

    if len(angles) < 8:
        print("FRONT_CALIBRATION_FAILED insufficient_samples")
        return 2

    front_deg = angle_mean_deg(angles)
    config = {
        "front_deg": front_deg,
        "sample_count": len(angles),
        "median_distance_m": statistics.median(distances),
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    with open(OUTPUT_PATH, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)
        f.write("\n")

    print(f"FRONT_CALIBRATION_OK front_deg={front_deg:.1f} samples={len(angles)}")
    print(f"SAVED {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
