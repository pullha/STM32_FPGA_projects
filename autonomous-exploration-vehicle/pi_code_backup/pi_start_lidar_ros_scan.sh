#!/usr/bin/env bash
set -euo pipefail

ROSCORE_LOG=/tmp/roscore.log
LIDAR_LOG=/tmp/ydlidar_scan_publisher.log

if ! pgrep -f "rosmaster --core" >/dev/null; then
  nohup roscore >"${ROSCORE_LOG}" 2>&1 </dev/null &
  sleep 3
fi

if ! pgrep -f "/home/pi/ydlidar_ros1_scan_publisher.py" >/dev/null; then
  nohup env PYTHONPATH=/home/pi/ros_genpy:/home/pi/YDLidar-SDK/build/python \
    /usr/bin/python3 /home/pi/ydlidar_ros1_scan_publisher.py \
    >"${LIDAR_LOG}" 2>&1 </dev/null &
  sleep 5
fi

rostopic list
