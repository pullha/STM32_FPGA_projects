# SLAM and object mapping

The existing ROS 1 occupancy-grid scan matcher and HSV/contour detector are
connected in `slam_object_mapper.py`. No camera command is sent to the motors.
The existing driving script and STM32 firmware are not modified by this change.

## Run on Raspberry Pi

Stationary capture (no motor CAN frames):

```sh
/usr/bin/python3 /home/pi/pi_run_exploration.py --seconds 20
```

Supervised driving with mapping and camera, for 180 seconds:

```sh
/usr/bin/python3 /home/pi/pi_run_exploration.py --drive --seconds 180
```

The runner waits for live camera frames and a nonempty map before starting the
drive process. It ends the session if either stops updating. On a drive-session
exit, it terminates the drive process and submits three zero-wheel STOP frames.
CAN submission is not proof of STM32 reception; the existing 500 ms motor-command
timeout remains necessary for loss of communication. Do not run a second drive
script alongside this runner. Each run uses a new result directory.

The runner stops its mapper/camera at the end but leaves an existing ROS master
and lidar publisher running. The original start/stop scripts are still available.

## Results

`RESULT_DIR` printed at startup contains:

- `map_latest.png`: full map with robot and object estimates.
- `map_preview.png`: cropped, enlarged map for inspection.
- `camera_debug_latest.jpg`: latest image and detected contours.
- `map.pgm` and `map.yaml`: occupancy grid for ROS map loading.
- `objects.json`: object IDs, positions in meters, last-seen times and uncertainty.
- `status.json`, `mapper.log`, `drive.log`: health and runtime diagnostics.

The map frame starts at the initial lidar position. Positive X is the calibrated
vehicle front; positive Y is left. Map pixels and ROS occupancy cells use their
respective top-left and bottom-left origins consistently. `/scan` remains raw
for the driving script; `/scan/map` is corrected and uses TF frame `laser_map`.

## Calibration and limitations

Measured raw angles are front=-97, right=-12, left=+175 degrees. Mapping therefore
uses `base_angle = -(raw_angle - front_angle)` with angle wrapping. This sign
correction cannot be represented by a yaw-only TF for the original raw scan.

The camera defaults to `cv2.VideoCapture(0)`, with UVC path fallback, and 640x480.
Three object IDs are preserved: 1 red sphere, 2 green cube, 3 blue prism.
Five consecutive processed frames confirm a detection. Missing detections reset
confirmation; one persistent marker per object ID prevents marker duplication.

Camera horizontal FOV=62 degrees, camera planar offset=(0, 0), and yaw=0 are
provisional defaults. The camera is on level 2 below the level-3 lidar. A lidar
return along the camera bearing can hit background above the object. All such
positions are explicitly marked `estimated`, with `height_match_verified=false`.
No bbox-size depth guess is used when a valid lidar return is unavailable;
the object is then recorded as detected but unlocated. Actual camera calibration
and measured object dimensions or a depth sensor are needed for reliable
positions when the object does not intersect the lidar plane.

ROS private parameters support `front_deg`, `angle_sign`, `camera_fov_deg`,
`camera_x_m`, `camera_y_m`, `camera_yaw_deg`, `confirm_frames`, `sensor_timeout`,
`output_dir`, `resolution`, and `size`. Start the mapper directly with these
parameters when calibrating; the runner exposes `--front-deg` and `--output-dir`.

This retains the project's lightweight scan-matching mapper. It has no loop
closure or encoder odometry input and can drift on a long run or fast rotation.
Successful stationary mapping does not verify moving-map or object accuracy.

## Checks

```sh
cd /home/pi/slam_integration_20260906
/usr/bin/python3 -m unittest -v test_slam_integration
```

Original local mapper backup: `outputs/slam_before_integration_20260906/`.
Original Pi mapper backup: `/home/pi/slam_integration_20260906/slam_object_mapper.before.py`.
