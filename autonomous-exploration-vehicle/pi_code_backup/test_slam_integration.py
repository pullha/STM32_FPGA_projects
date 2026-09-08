import math
import tempfile
import threading
import time
import unittest
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
import rospy

from slam_geometry import associate_lidar, base_to_map, camera_bearing, raw_to_base_angle
from slam_object_mapper import SimpleSlamObjectMapper
from pi_run_exploration import healthy, stop_motors


def scan_with_returns(returns):
    ranges = [float("inf")] * 360
    for degrees, distance in returns:
        ranges[(int(degrees) + 180) % 360] = distance
    return SimpleNamespace(ranges=ranges, range_min=0.12, range_max=5.0,
                           angle_min=-math.pi, angle_increment=math.pi / 180.0,
                           header=SimpleNamespace(stamp=rospy.Time(1)))


class GeometryTests(unittest.TestCase):
    def test_measured_front_right_left(self):
        self.assertAlmostEqual(raw_to_base_angle(math.radians(-97)), 0)
        self.assertAlmostEqual(math.degrees(raw_to_base_angle(math.radians(-12))), -85)
        self.assertAlmostEqual(math.degrees(raw_to_base_angle(math.radians(175))), 88)

    def test_camera_left_is_base_left(self):
        self.assertGreater(camera_bearing(100, 640, math.radians(62)), 0)
        self.assertLess(camera_bearing(540, 640, math.radians(62)), 0)
        self.assertEqual(camera_bearing(320, 640, math.radians(62)), 0)

    def test_front_uses_calibrated_sector_not_raw_zero(self):
        point = associate_lidar(scan_with_returns([(-97, 0.8), (0, 4.0)]), 0, -97, -1)
        self.assertAlmostEqual(point[0], 0.8)
        self.assertAlmostEqual(point[1], 0)

    def test_angle_wrap_and_camera_offset(self):
        scan = scan_with_returns([(179, 1), (-179, 1)])
        self.assertIsNotNone(associate_lidar(scan, math.radians(83), -97, -1))
        scan = scan_with_returns([(-97, 1)])
        point = associate_lidar(scan, math.atan2(-0.1, 0.8), -97, -1, 0.2, 0.1)
        self.assertAlmostEqual(point[0], 1.0)

    def test_invalid_ranges_do_not_fabricate_positions(self):
        scan = scan_with_returns([(-97, float("nan")), (-96, 0), (-98, 8)])
        self.assertIsNone(associate_lidar(scan, 0, -97, -1))

    def test_world_transform(self):
        x, y = base_to_map((2, 0), (3, 4, math.pi / 2))
        self.assertAlmostEqual(x, 3)
        self.assertAlmostEqual(y, 6)


class MapperTests(unittest.TestCase):
    def setUp(self):
        self.mapper = SimpleSlamObjectMapper.__new__(SimpleSlamObjectMapper)
        self.mapper.front_deg, self.mapper.angle_sign = -97, -1
        self.mapper.size, self.mapper.resolution = 20, 0.05
        self.mapper.origin_x, self.mapper.origin_y = -0.5, -0.5
        self.mapper.logodds = np.zeros((20, 20), np.float32)
        self.mapper.pose = np.array([0, 0, 0], np.float32)
        self.mapper.object_counts = defaultdict(int)

    def test_ros_occupancy_origin_not_image_origin(self):
        self.mapper.logodds[2, 3] = 2
        cells = self.mapper.occupancy_data()
        self.assertEqual(cells[2 * 20 + 3], 100)
        self.assertEqual(cells[17 * 20 + 3], -1)

    def test_below_origin_is_outside(self):
        self.assertFalse(self.mapper.cell_inside(*self.mapper.world_to_cell(-0.501, 0)))

    def test_mapper_scan_points_calibrated(self):
        points = self.mapper.scan_points(scan_with_returns([(-97, 1)]), step=1)
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][1], 0)

    def test_red_green_blue_and_noise(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        cv2.circle(frame, (320, 240), 24, (0, 0, 255), -1)
        cv2.rectangle(frame, (130, 210), (190, 270), (0, 255, 0), -1)
        cv2.fillPoly(frame, [np.array([[480, 210], [450, 270], [510, 270]])], (255, 0, 0))
        self.assertEqual({d[0] for d in self.mapper.detect_objects(frame)}, {1, 2, 3})
        frame[:] = 0
        cv2.circle(frame, (200, 200), 3, (0, 0, 255), -1)
        self.assertEqual(self.mapper.detect_objects(frame), [])

    def test_edge_clipped_blue_background_is_not_a_prism(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(frame, (0, 175), (90, 220), (255, 0, 0), -1)
        self.assertEqual(self.mapper.detect_objects(frame), [])

    def test_clear_ray_has_no_alternating_unknown_cells(self):
        self.mapper.integrate_scan([(0.4, 0.0)])
        start_x, start_y = self.mapper.world_to_cell(0, 0)
        end_x, _ = self.mapper.world_to_cell(0.4, 0)
        self.assertTrue(np.all(self.mapper.logodds[start_y, start_x:end_x] < 0))

    def test_outputs_are_real_map_files_and_estimate_metadata(self):
        m = self.mapper
        m.logodds[4, 5] = 2
        m.logodds[4, 4] = -1
        m.object_positions = {1: (0.1, 0.2, 1.0)}
        m.object_metadata = {1: {"estimated": True}}
        m.current_detections = []
        m.map_frame = "map"
        m.camera_fov_rad = math.radians(62)
        m.camera_x = m.camera_y = m.camera_yaw = 0
        m.scan_count = 3
        m.frame_index = 15
        m.enable_camera = True
        m.last_camera_time = time.monotonic()
        with tempfile.TemporaryDirectory() as directory:
            m.output_dir = directory
            m.save_map_png()
            files = {p.name for p in Path(directory).iterdir()}
            self.assertTrue({"map.pgm", "map.yaml", "map_latest.png", "map_preview.png",
                             "objects.json", "status.json"} <= files)
            self.assertIn('"estimated": true', (Path(directory) / "objects.json").read_text())
            pgm = cv2.imread(str(Path(directory) / "map.pgm"), cv2.IMREAD_GRAYSCALE)
            self.assertEqual(int(pgm[15, 5]), 20)

    def run_camera_frames(self, frame_count, position):
        m = self.mapper
        m.object_counts = defaultdict(int)
        m.frame_index = 0
        m.confirm_frames = 5
        m.sensor_timeout = 0.5
        m.last_processed_frame_time = time.monotonic()
        m.last_camera_debug_time = time.time()
        m.camera_fov_rad, m.camera_yaw = math.radians(62), 0
        m.latest_scan_lock = threading.RLock()
        m.object_positions, m.object_metadata = {}, {}
        m.current_detections = []
        m.range_at_bearing = Mock(return_value=position)
        m.write_image = Mock()
        frame = np.zeros((480, 640, 3), np.uint8)
        cv2.circle(frame, (320, 240), 24, (0, 0, 255), -1)
        capture = Mock()
        capture.read.return_value = (True, frame)
        m.open_camera = Mock(return_value=(capture, 640))
        with patch("slam_object_mapper.rospy.is_shutdown", side_effect=lambda: m.frame_index >= frame_count), \
                patch("slam_object_mapper.rospy.loginfo_throttle"):
            m.camera_loop()
        capture.release.assert_called_once()

    def test_confirmation_and_single_marker_per_object(self):
        self.run_camera_frames(12, (0.8, 0))
        self.assertEqual(self.mapper.object_positions, {})
        self.run_camera_frames(24, (0.8, 0))
        self.assertEqual(set(self.mapper.object_positions), {1})
        self.assertTrue(self.mapper.current_detections[0]["confirmed"])

    def test_confirmed_detection_without_depth_stays_unlocated(self):
        self.run_camera_frames(18, None)
        self.assertEqual(self.mapper.object_positions, {})
        self.assertTrue(self.mapper.current_detections[0]["confirmed"])
        self.assertEqual(self.mapper.current_detections[0]["position_status"], "unlocated")


class SessionTests(unittest.TestCase):
    def test_missing_camera_or_stale_results_prevent_driving(self):
        status = {"saved_at": time.time(), "scan_count": 10, "known_cells": 100, "camera_ok": True}
        self.assertTrue(healthy(status))
        self.assertFalse(healthy(dict(status, camera_ok=False)))
        self.assertFalse(healthy(dict(status, saved_at=time.time() - 10)))

    def test_cleanup_sends_only_zero_wheel_stop(self):
        import struct
        with patch("pi_run_exploration.socket.socket") as socket_mock:
            stop_motors()
        bus = socket_mock.return_value.__enter__.return_value
        self.assertEqual(bus.send.call_count, 3)
        can_id, dlc, payload = struct.unpack("=IB3x8s", bus.send.call_args.args[0])
        self.assertEqual((can_id, dlc, payload), (0x210, 4, bytes(8)))


if __name__ == "__main__":
    unittest.main()
