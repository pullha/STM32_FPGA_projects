import cv2
import numpy as np

# USB 카메라 열기
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Camera Open Failed")
    raise SystemExit

while True:
    ret, frame = cap.read()

    if not ret:
        print("Frame Read Failed")
        break

    # 영상 크기를 줄여서 처리 속도 향상
    frame = cv2.resize(frame, (640, 480))

    # BGR 영상을 HSV 색공간으로 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 빨간색은 HSV에서 양 끝 구간으로 나뉨
    lower_red1 = np.array([0, 100, 80])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 100, 80])
    upper_red2 = np.array([179, 255, 255])

    # 빨간색 영역 추출
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # 작은 노이즈 제거
    kernel = np.ones((5, 5), np.uint8)

    red_mask = cv2.morphologyEx(
        red_mask,
        cv2.MORPH_OPEN,
        kernel
    )

    red_mask = cv2.morphologyEx(
        red_mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    # 빨간색 외곽선 검출
    contours, _ = cv2.findContours(
        red_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    red_ball_detected = False

    for contour in contours:
        area = cv2.contourArea(contour)

        # 너무 작은 빨간 영역은 무시
        if area < 1000:
            continue

        perimeter = cv2.arcLength(contour, True)

        if perimeter == 0:
            continue

        # 원형도 계산
        # 완전한 원에 가까울수록 1에 가까움
        circularity = 4 * np.pi * area / (perimeter * perimeter)

        # 검출 영역의 최소 외접원
        (x, y), radius = cv2.minEnclosingCircle(contour)

        center = (int(x), int(y))
        radius = int(radius)

        # 빨간색이고 충분히 원형일 때 빨간 공으로 판정
        if circularity > 0.65 and radius > 15:
            red_ball_detected = True

            # 공 주변에 원 표시
            cv2.circle(
                frame,
                center,
                radius,
                (0, 255, 0),
                3
            )

            # 중심점 표시
            cv2.circle(
                frame,
                center,
                5,
                (255, 255, 255),
                -1
            )

            # 정보 표시
            cv2.putText(
                frame,
                "RED BALL",
                (center[0] - 70, center[1] - radius - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                "Object ID : 1",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"Circularity : {circularity:.2f}",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            # 가장 큰 빨간 공 하나만 처리
            break

    if not red_ball_detected:
        cv2.putText(
            frame,
            "RED BALL NOT FOUND",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

    cv2.imshow("Red Ball Detection", frame)
    cv2.imshow("Red Mask", red_mask)

    # q 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
