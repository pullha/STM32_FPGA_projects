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

    # 초록색 HSV 범위
    lower_green = np.array([35, 80, 60], dtype=np.uint8)
    upper_green = np.array([85, 255, 255], dtype=np.uint8)

    green_mask = cv2.inRange(
        hsv,
        lower_green,
        upper_green
    )

    # 작은 노이즈 제거
    green_mask = cv2.morphologyEx(
        green_mask,
        cv2.MORPH_OPEN,
        kernel
    )

    # 물체 내부의 작은 빈 공간 채우기
    green_mask = cv2.morphologyEx(
        green_mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    contours, _ = cv2.findContours(
        green_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    green_cube_detected = False

    # 큰 외곽선부터 검사
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

        # 외곽선을 단순화하여 꼭짓점 수 계산
        approx = cv2.approxPolyDP(
            contour,
            0.04 * perimeter,
            True
        )

        vertex_count = len(approx)

        x, y, width, height = cv2.boundingRect(approx)

        if height == 0:
            continue

        aspect_ratio = width / float(height)

        # 정육면체는 카메라에서 대체로 사각형으로 보임
        # 원근 때문에 꼭짓점이 4~6개로 검출될 수 있음
        if 4 <= vertex_count <= 6 and 0.60 <= aspect_ratio <= 1.50:
            green_cube_detected = True

            cv2.drawContours(
                frame,
                [approx],
                -1,
                (0, 255, 0),
                3
            )

            cv2.rectangle(
                frame,
                (x, y),
                (x + width, y + height),
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                "GREEN CUBE",
                (x, max(y - 15, 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                "Object ID : 2",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"Vertices : {vertex_count}",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            break

    if not green_cube_detected:
        cv2.putText(
            frame,
            "GREEN CUBE NOT FOUND",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

    cv2.imshow("Green Cube Detection", frame)
    cv2.imshow("Green Mask", green_mask)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


cap.release()
cv2.destroyAllWindows()
