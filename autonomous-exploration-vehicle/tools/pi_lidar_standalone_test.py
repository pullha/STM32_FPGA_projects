#!/usr/bin/env python3
import math
import statistics
import sys
import time

sys.path.insert(0, "/home/pi/YDLidar-SDK/build/python")

import ydlidar


def main():
    port = "/dev/ttyUSB0"
    laser = ydlidar.CYdLidar()

    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
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

    print("LIDAR_STARTED")
    scan = ydlidar.LaserScan()
    good_scans = 0

    try:
        deadline = time.time() + 10.0
        while time.time() < deadline and good_scans < 5:
            if not laser.doProcessSimple(scan):
                time.sleep(0.05)
                continue

            ranges = [
                p.range
                for p in scan.points
                if math.isfinite(p.range) and 0.12 <= p.range <= 10.0
            ]
            if not ranges:
                print(f"SCAN_EMPTY total_points={len(scan.points)}")
                continue

            good_scans += 1
            print(
                "SCAN_OK "
                f"idx={good_scans} "
                f"total_points={len(scan.points)} "
                f"valid_points={len(ranges)} "
                f"min_m={min(ranges):.3f} "
                f"median_m={statistics.median(ranges):.3f} "
                f"max_m={max(ranges):.3f}"
            )
    finally:
        laser.turnOff()
        laser.disconnecting()
        print("LIDAR_STOPPED")

    if good_scans == 0:
        print("LIDAR_NO_VALID_SCAN")
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
