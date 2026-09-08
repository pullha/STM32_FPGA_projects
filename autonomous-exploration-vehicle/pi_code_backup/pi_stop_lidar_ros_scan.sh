#!/usr/bin/env bash
set -euo pipefail

pkill -f "/home/pi/ydlidar_ros1_scan_publisher.py" 2>/dev/null || true
pkill -f "rosmaster --core" 2>/dev/null || true
pkill -f "/usr/bin/roscore" 2>/dev/null || true

echo "Stopped lidar ROS scan publisher and roscore"
