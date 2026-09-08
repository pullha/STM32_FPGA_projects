import cv2
import numpy as np


CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MIN_AREA = 1000


cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print("Camera Open Failed")
    raise SystemExit


kernel = np.ones((5, 5), np.uint8)


while True:
    ret, frame = cap.read()

    if not ret:
        print("Frame Read Failed")
        break

    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

    # BGR -> HSV 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 파란색 HSV 범위
    lower_blue = np.array([90, 80, 60], dtype=np.uint8)
    upper_blue = np.array([135, 255, 255], dtype=np.uint8)

    blue_mask = cv2.inRange(
        hsv,
        lower_blue,
        upper_blue
    )

    # 작은 노이즈 제거
    blue_mask = cv2.morphologyEx(
        blue_mask,
        cv2.MORPH_OPEN,
        kernel
    )

    # 물체 내부 빈 공간 채우기
    blue_mask = cv2.morphologyEx(
        blue_mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    contours, _ = cv2.findContours(
        blue_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    blue_prism_detected = False

    contours = sorted(
        contours,
        key=cv2.contourArea,
        reverse=True
    )

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < MIN_AREA:
            continue

        perimeter = cv2.arcLength(contour, True)

        if perimeter == 0:
            continue

        approx = cv2.approxPolyDP(
            contour,
            0.04 * perimeter,
            True
        )

        vertex_count = len(approx)

        # 삼각기둥의 삼각형 면을 검출
        if vertex_count == 3:
            blue_prism_detected = True

            x, y, width, height = cv2.boundingRect(approx)

            cv2.drawContours(
                frame,
                [approx],
                -1,
                (0, 255, 255),
                3
            )

            cv2.rectangle(
                frame,
                (x, y),
                (x + width, y + height),
                (0, 255, 255),
                2
            )

            cv2.putText(
                frame,
                "BLUE TRIANGULAR PRISM",
                (x, max(y - 15, 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2
            )

            cv2.putText(
                frame,
                "Object ID : 3",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f"Vertices : {vertex_count}",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

            break

    if not blue_prism_detected:
        cv2.putText(
            frame,
            "BLUE PRISM NOT FOUND",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

    cv2.imshow("Blue Prism Detection", frame)
    cv2.imshow("Blue Mask", blue_mask)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


cap.release()
cv2.destroyAllWindows()
