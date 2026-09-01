#!/usr/bin/env bash
set -euo pipefail

pkill -f "/home/pi/slam_object_mapper.py" 2>/dev/null || true
/home/pi/pi_stop_lidar_ros_scan.sh >/tmp/stop_lidar_ros_scan.last.log 2>&1 || true

echo "Stopped SLAM/object mapper and lidar ROS scan publisher"
