#!/usr/bin/env python3
import math
import statistics
import sys
import time

sys.path.insert(0, "/home/pi/YDLidar-SDK/build/python")
import ydlidar


def median_in_sector(scan, center_deg, width_deg):
    values = []
    half = math.radians(width_deg / 2.0)
    center = math.radians(center_deg)

    for point in scan.points:
        distance = point.range
        if not math.isfinite(distance) or distance < 0.12 or distance > 6.0:
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


def closest_angle(scan):
    best_angle = None
    best_distance = float("inf")

    for point in scan.points:
        distance = point.range
        if not math.isfinite(distance) or distance < 0.12 or distance > 6.0:
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

    print("LIDAR_ORIENTATION_CHECK_START")
    print("Put your hand/object close to the robot's physical front.")
    scan = ydlidar.LaserScan()

    try:
        end_time = time.time() + 12.0
        while time.time() < end_time:
            if not laser.doProcessSimple(scan):
                time.sleep(0.05)
                continue

            near_angle, near_distance = closest_angle(scan)
            sectors = {
                "0deg": median_in_sector(scan, 0, 25),
                "+45deg": median_in_sector(scan, 45, 25),
                "+90deg": median_in_sector(scan, 90, 25),
                "+135deg": median_in_sector(scan, 135, 25),
                "180deg": median_in_sector(scan, 180, 25),
                "-135deg": median_in_sector(scan, -135, 25),
                "-90deg": median_in_sector(scan, -90, 25),
                "-45deg": median_in_sector(scan, -45, 25),
            }
            sector_text = " ".join(
                f"{name}={distance:.2f}m" for name, distance in sectors.items()
            )
            print(
                f"closest={near_angle:.1f}deg/{near_distance:.2f}m {sector_text}",
                flush=True,
            )
            time.sleep(0.4)
    finally:
        laser.turnOff()
        laser.disconnecting()
        print("LIDAR_ORIENTATION_CHECK_STOP")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
