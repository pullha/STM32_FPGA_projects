"""Coordinate helpers shared by the mapper and hardware-free tests."""
import math
import statistics


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def raw_to_base_angle(angle, front_deg=-97.0, angle_sign=-1.0):
    # Hand calibration: front=-97, right=-12, left=+175 degrees.
    return wrap_angle(angle_sign * (angle - math.radians(front_deg)))


def camera_bearing(center_x, width, fov_rad):
    focal_px = width / (2.0 * math.tan(fov_rad / 2.0))
    return math.atan2(width / 2.0 - center_x, focal_px)


def associate_lidar(scan, bearing, front_deg, angle_sign, camera_x=0.0,
                    camera_y=0.0, half_window=math.radians(3.0), max_range=5.0):
    """Return a scan endpoint in base coordinates, not a fabricated depth.

    This is a 2-D bearing association. The endpoint can be background when
    the target is below the lidar plane, so callers must label it estimated.
    """
    matches = []
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= min(scan.range_max, max_range):
            continue
        angle = raw_to_base_angle(scan.angle_min + i * scan.angle_increment,
                                  front_deg, angle_sign)
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        ray_angle = math.atan2(y - camera_y, x - camera_x)
        if abs(wrap_angle(ray_angle - bearing)) <= half_window:
            matches.append((math.hypot(x - camera_x, y - camera_y), x, y))
    if not matches:
        return None
    median_range = statistics.median(item[0] for item in matches)
    return min(matches, key=lambda item: abs(item[0] - median_range))[1:]


def base_to_map(point, pose):
    x, y = point
    px, py, yaw = pose
    return (px + math.cos(yaw) * x - math.sin(yaw) * y,
            py + math.sin(yaw) * x + math.cos(yaw) * y)
