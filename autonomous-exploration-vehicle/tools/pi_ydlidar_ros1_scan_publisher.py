#!/usr/bin/env python3
import math
import sys

sys.path.insert(0, "/home/pi/YDLidar-SDK/build/python")
sys.path.insert(0, "/home/pi/ros_genpy")

import rospy
from sensor_msgs.msg import LaserScan
import ydlidar


def set_lidar_options(laser, port, baudrate, min_range, max_range, scan_frequency):
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 5)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, scan_frequency)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, max_range)
    laser.setlidaropt(ydlidar.LidarPropMinRange, min_range)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)


def scan_to_msg(scan, frame_id, min_range, max_range):
    msg = LaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    msg.angle_min = scan.config.min_angle
    msg.angle_max = scan.config.max_angle
    msg.angle_increment = scan.config.angle_increment
    msg.time_increment = scan.config.time_increment
    msg.scan_time = scan.config.scan_time
    msg.range_min = min_range
    msg.range_max = max_range

    count = int(math.ceil((msg.angle_max - msg.angle_min) / msg.angle_increment))
    count = max(count, 1)
    msg.ranges = [float("inf")] * count
    msg.intensities = [0.0] * count

    for point in scan.points:
        if not math.isfinite(point.range):
            continue
        if point.range < min_range or point.range > max_range:
            continue

        index = int(math.ceil((point.angle - msg.angle_min) / msg.angle_increment))
        if 0 <= index < count:
            msg.ranges[index] = point.range
            msg.intensities[index] = getattr(point, "intensity", 0.0)

    return msg


def main():
    rospy.init_node("ydlidar_ros1_scan_publisher")

    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = int(rospy.get_param("~baudrate", 128000))
    frame_id = rospy.get_param("~frame_id", "laser")
    min_range = float(rospy.get_param("~min_range", 0.12))
    max_range = float(rospy.get_param("~max_range", 10.0))
    scan_frequency = float(rospy.get_param("~scan_frequency", 10.0))

    laser = ydlidar.CYdLidar()
    set_lidar_options(laser, port, baudrate, min_range, max_range, scan_frequency)

    if not laser.initialize():
        rospy.logerr("Failed to initialize YDLidar on %s", port)
        return 1

    if not laser.turnOn():
        rospy.logerr("Failed to turn on YDLidar")
        laser.disconnecting()
        return 1

    publisher = rospy.Publisher("/scan", LaserScan, queue_size=5)
    scan = ydlidar.LaserScan()
    rospy.loginfo("Publishing YDLidar scans on /scan from %s", port)

    try:
        while not rospy.is_shutdown():
            if not laser.doProcessSimple(scan):
                rospy.sleep(0.02)
                continue

            msg = scan_to_msg(scan, frame_id, min_range, max_range)
            publisher.publish(msg)
    finally:
        laser.turnOff()
        laser.disconnecting()
        rospy.loginfo("YDLidar stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
