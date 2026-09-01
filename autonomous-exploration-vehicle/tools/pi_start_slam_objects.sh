#!/usr/bin/env bash
set -euo pipefail

/home/pi/pi_start_lidar_ros_scan.sh >/tmp/start_lidar_ros_scan.last.log 2>&1

if ! pgrep -f "/home/pi/slam_object_mapper.py" >/dev/null; then
  nohup /usr/bin/python3 /home/pi/slam_object_mapper.py \
    > /tmp/slam_object_mapper.log 2>&1 </dev/null &
  sleep 5
fi

rostopic list
