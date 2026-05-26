# vision_test.py
# ArUco + 색상 인식 통합 테스트
# 물체의 로봇팔 좌표 계산 확인

import cv2
import numpy as np
import config_vision as cfg

URL = 'http://192.0.0.3:8081/video'

# ArUco 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params     = cv2.aruco.DetectorParameters()
detector   = cv2.aruco.ArucoDetector(aruco_dict, params)

cap = cv2.VideoCapture(URL)
print('통합 테스트 시작')
print('ArUco 마커 + 파란 물체를 카메라에 비춰주세요')
print('q: 종료')

def detect_aruco(frame):
    """ArUco 마커 감지 → 스케일, 원점 반환"""
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return None, None

    markers = {}
    for i, mid in enumerate(ids.flatten()):
        c  = corners[i][0]
        cx = int(c[:, 0].mean())
        cy = int(c[:, 1].mean())
        markers[int(mid)] = (cx, cy, c)

    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    if 0 not in markers:
        return None, None

    # 픽셀당 cm 계산
    c         = markers[0][2]
    side_px   = np.linalg.norm(c[0] - c[1])
    px_per_cm = side_px / cfg.MARKER_CM
    origin_px = (markers[0][0], markers[0][1])

    return px_per_cm, origin_px

def detect_blue(frame):
    """파란 물체 감지 → 중심 픽셀 반환"""
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(cfg.COLOR_LOWER),
        np.array(cfg.COLOR_UPPER)
    )

    # 노이즈 제거
    kernel = np.ones((5, 5), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 500:
        return None

    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return cx, cy

def pixel_to_robot(px, py, px_per_cm, origin_px):
    """픽셀 좌표 → 로봇팔 좌표 변환"""
    dx_cm = (px - origin_px[0]) / px_per_cm
    dy_cm = (py - origin_px[1]) / px_per_cm

    robot_x = cfg.BASE_OFFSET_X - dy_cm
    #robot_y = -(dx_cm + cfg.BASE_OFFSET_Y)
    robot_y = (dx_cm + cfg.BASE_OFFSET_Y)
    return round(robot_x, 1), round(robot_y, 1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]

    # ArUco 감지
    px_per_cm, origin_px = detect_aruco(frame)

    # 파란 물체 감지
    obj_px   = detect_blue(frame)
    robot_xy = None

    if px_per_cm and origin_px:
        # 스케일 표시
        cv2.putText(frame,
            f'Scale: {px_per_cm:.1f} px/cm',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 0), 2)

        # 원점 표시
        cv2.circle(frame, origin_px, 8, (0, 255, 0), -1)
        cv2.putText(frame, 'Origin',
            (origin_px[0]+10, origin_px[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 1)

        if obj_px:
            # 물체 표시
            cv2.circle(frame, obj_px, 10, (0, 0, 255), -1)

            # 로봇 좌표 계산
            rx, ry   = pixel_to_robot(
                obj_px[0], obj_px[1],
                px_per_cm, origin_px
            )
            robot_xy = (rx, ry)

            # 좌표 표시
            cv2.putText(frame,
                f'로봇 좌표: ({rx}, {ry}) cm',
                (obj_px[0]+12, obj_px[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 255), 2)

            # 원점 → 물체 선 연결
            cv2.line(frame, origin_px, obj_px,
                     (255, 0, 255), 1)

            print(f'물체 로봇 좌표: ({rx}, {ry}) cm')

    # 상태 표시
    if px_per_cm is None:
        cv2.putText(frame,
            'ArUco 마커를 비춰주세요',
            (10, h-40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 0, 255), 2)
    elif obj_px is None:
        cv2.putText(frame,
            '파란 물체를 비춰주세요',
            (10, h-40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 165, 255), 2)
    else:
        cv2.putText(frame,
            f'물체 감지! ({robot_xy[0]}, {robot_xy[1]}) cm',
            (10, h-40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 255, 0), 2)

    cv2.imshow('Vision Test', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
