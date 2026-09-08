#!/usr/bin/env python3
"""Run mapping/camera capture; add --drive only for a supervised drive run."""
import argparse
import fcntl
import json
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent


def stop_motors():
    frame = struct.pack("=IB3x8s", 0x210, 4, bytes(8))
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.settimeout(0.3)
        bus.bind(("can0",))
        for _ in range(3):
            bus.send(frame)


def terminate(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=4)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)


def read_status(directory):
    try:
        return json.loads((directory / "status.json").read_text())
    except (OSError, ValueError):
        return {}


def healthy(status):
    return (time.time() - status.get("saved_at", 0.0) < 3.0
            and status.get("scan_count", 0) >= 3
            and status.get("known_cells", 0) > 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=180.0)
    parser.add_argument("--drive", action="store_true")
    parser.add_argument("--front-deg", type=float, default=-97.0)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if not 0 < args.seconds <= 1800:
        parser.error("--seconds must be between 0 and 1800")
    session_lock = (ROOT / ".exploration.lock").open("a")
    try:
        fcntl.flock(session_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.exit(1, "An exploration session is already running\n")
    existing = subprocess.run(["pgrep", "-af", "slam_object_mapper[.]py|ros_scan_can_drive"],
                              capture_output=True, text=True, timeout=3)
    if existing.returncode == 0:
        parser.exit(1, "Stop the existing mapper/drive first:\n" + existing.stdout)
    directory = args.output_dir or (ROOT / "slam_outputs" / time.strftime("run_%Y%m%d_%H%M%S"))
    directory.mkdir(parents=True, exist_ok=False)
    print("RESULT_DIR=" + str(directory), flush=True)

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    mapper = drive = None
    code = 0
    with (directory / "mapper.log").open("w") as mapper_log, \
            (directory / "drive.log").open("w") as drive_log, \
            (directory / "lidar_start.log").open("w") as lidar_log:
        try:
            if args.drive:
                stop_motors()
                link = subprocess.run(["ip", "-details", "link", "show", "can0"],
                                      capture_output=True, text=True, check=True, timeout=3)
                if "ERROR-ACTIVE" not in link.stdout:
                    raise RuntimeError("CAN is not ERROR-ACTIVE; drive was not started")
            subprocess.run(["bash", str(ROOT / "pi_start_lidar_ros_scan.sh")],
                           stdout=lidar_log, stderr=subprocess.STDOUT, check=True, timeout=20)
            mapper = subprocess.Popen([
                "/usr/bin/python3", "-u", str(ROOT / "slam_object_mapper.py"),
                "_output_dir:=" + str(directory), "_front_deg:=" + str(args.front_deg),
            ], stdout=mapper_log, stderr=subprocess.STDOUT)
            ready_by = time.monotonic() + 25.0
            while not healthy(read_status(directory)):
                if mapper.poll() is not None or time.monotonic() > ready_by:
                    raise RuntimeError("Map/camera did not become ready; see mapper.log")
                time.sleep(0.2)
            print("READY: map and camera data are updating", flush=True)
            if args.drive:
                code_text = ("import ros_scan_can_drive as m; "
                             f"m.RUN_SECONDS={args.seconds!r}; raise SystemExit(m.main())")
                drive = subprocess.Popen(["/usr/bin/python3", "-u", "-c", code_text,
                                          "_front_deg:=" + str(args.front_deg)],
                                         cwd=ROOT, stdout=drive_log, stderr=subprocess.STDOUT)
            start = time.monotonic()
            next_report = start
            while time.monotonic() - start < args.seconds:
                status = read_status(directory)
                if mapper.poll() is not None or not healthy(status):
                    raise RuntimeError("Map or camera stopped updating; ending session")
                if drive is not None and drive.poll() is not None:
                    if drive.returncode != 0 or time.monotonic() - start < args.seconds - 1.0:
                        raise RuntimeError("Drive process ended early; see drive.log")
                    break
                if time.monotonic() >= next_report:
                    print("RUN elapsed=%.0fs scans=%s camera_frames=%s objects=%s" % (
                        time.monotonic() - start, status.get("scan_count"),
                        status.get("camera_frames"), status.get("mapped_objects")), flush=True)
                    next_report += 10.0
                time.sleep(0.2)
        except KeyboardInterrupt:
            code = 130
            print("Interrupted; ending session", flush=True)
        except (OSError, RuntimeError, subprocess.SubprocessError) as error:
            code = 1
            print("ERROR: " + str(error), flush=True)
        finally:
            terminate(drive)
            if args.drive:
                try:
                    stop_motors()
                    print("STOP submitted to can0", flush=True)
                except OSError as error:
                    code = 1
                    print("STOP send failed: " + str(error), flush=True)
            terminate(mapper)
    print("FINISHED result=" + str(directory), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
