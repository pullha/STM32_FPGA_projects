#!/usr/bin/env python3
import math
import sys

import cv2
import numpy as np


OBJECTS = {
    "RED SPHERE": (
        (40, 40, 230),
        lambda hsv: cv2.bitwise_or(
            cv2.inRange(hsv, (0, 120, 100), (8, 255, 255)),
            cv2.inRange(hsv, (172, 120, 100), (180, 255, 255)),
        ),
    ),
    "GREEN CUBE": ((40, 200, 40), lambda hsv: cv2.inRange(hsv, (35, 45, 35), (90, 255, 255))),
    "BLUE PRISM": ((230, 80, 40), lambda hsv: cv2.inRange(hsv, (90, 45, 35), (140, 255, 255))),
}


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/home/pi/slam_outputs/camera_debug_latest.jpg"
    frame = cv2.imread(path)
    if frame is None:
        print(f"FAILED_TO_READ {path}")
        return 1

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_h, frame_w = frame.shape[:2]
    kernel = np.ones((5, 5), np.uint8)
    print(f"IMAGE {path} shape={frame.shape}")

    for name, (_, mask_fn) in OBJECTS.items():
        mask = mask_fn(hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:5]
        print(f"{name}: contours={len(contours)}")
        for idx, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
            circularity = 0.0 if peri <= 0 else 4.0 * math.pi * area / (peri * peri)
            x, y, w, h = cv2.boundingRect(contour)
            aspect = w / float(h) if h > 0 else 0.0
            edge_touch = x <= 6 or y <= 6 or (x + w) >= frame_w - 6 or (y + h) >= frame_h - 6
            red_ok = (
                not edge_touch
                and 0.55 <= aspect <= 1.65
                and 22 <= min(w, h)
                and max(w, h) <= 170
                and 700 <= area <= 18000
                and circularity >= 0.26
            )
            print(
                f"  #{idx} area={area:.1f} vertices={len(approx)} "
                f"circularity={circularity:.3f} aspect={aspect:.2f} "
                f"edge={edge_touch} red_ok={red_ok} bbox=({x},{y},{w},{h})"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
