#!/usr/bin/env python3
import os

import cv2

OUT_DIR = "/home/pi/slam_outputs/camera_probe"
os.makedirs(OUT_DIR, exist_ok=True)


def main():
    for index in range(0, 32):
        path = f"/dev/video{index}"
        if not os.path.exists(path):
            continue

        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"{path}: open failed")
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ok, frame = cap.read()
        cap.release()

        if not ok or frame is None:
            print(f"{path}: read failed")
            continue

        out_path = os.path.join(OUT_DIR, f"video{index}.jpg")
        cv2.imwrite(out_path, frame)
        print(f"{path}: saved {out_path} shape={frame.shape}")


if __name__ == "__main__":
    main()
