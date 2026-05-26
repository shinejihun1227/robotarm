# main_vision.py
import cv2
import time
import threading
import numpy as np
import config_vision as cfg
from serial_comm import ArmSerial

print('=== 비전 제어 시작 ===')

# ── ESP32 연결 ───────────────────────────────────────────
arm = ArmSerial(cfg.SERIAL_PORT)

# ── 카메라 연결 ──────────────────────────────────────────
cap = cv2.VideoCapture(cfg.CAMERA_URL)
if not cap.isOpened():
    print('카메라 연결 실패')
    exit()
print('카메라 연결 완료')

# ── ArUco 설정 ───────────────────────────────────────────
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params     = cv2.aruco.DetectorParameters()
detector   = cv2.aruco.ArucoDetector(aruco_dict, params)

# ── 상태 변수 ────────────────────────────────────────────
is_running = False


# ── ArUco 감지 ───────────────────────────────────────────
def detect_aruco(frame):
    """
    ArUco 마커 감지
    반환: px_per_cm, origin_px
    """
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

    c         = markers[0][2]
    side_px   = np.linalg.norm(c[0] - c[1])
    px_per_cm = side_px / cfg.MARKER_CM
    origin_px = (markers[0][0], markers[0][1])

    return px_per_cm, origin_px


# ── 색상 감지 ────────────────────────────────────────────
def detect_object(frame):
    """
    파란 물체 감지
    반환: (cx, cy) 중심 픽셀 또는 None
    """
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(cfg.COLOR_LOWER),
        np.array(cfg.COLOR_UPPER)
    )

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


# ── 좌표 변환 ────────────────────────────────────────────
def pixel_to_robot(px, py, px_per_cm, origin_px):
    dx_cm = (px - origin_px[0]) / px_per_cm
    dy_cm = (py - origin_px[1]) / px_per_cm

    robot_x = cfg.BASE_OFFSET_X - dy_cm
    robot_y = dx_cm - cfg.BASE_OFFSET_Y  # 오프셋 빼기

    return round(robot_x, 1), round(robot_y, 1)


# ── 집기 동작 ────────────────────────────────────────────
def pick_and_place(robot_x, robot_y):
    """
    물체 위치로 접근 후 홈 복귀
    별도 스레드에서 실행
    """
    global is_running
    is_running = True

    print(f'\n접근 시작: ({robot_x:.1f}, {robot_y:.1f})')

    try:
        # 1. 물체 위 높은 위치로 이동
        print(f'  물체 위로 이동 (높이: {cfg.HOVER_Z}cm)...')
        arm.move_to(robot_x, robot_y, cfg.HOVER_Z)
        time.sleep(3.0)

        # 2. 중간 높이로
        mid_z = (cfg.HOVER_Z + cfg.PICK_Z) / 2
        print(f'  중간 높이로 (높이: {mid_z:.1f}cm)...')
        arm.move_to(robot_x, robot_y, mid_z)
        time.sleep(2.0)

        # 3. 목표 높이로
        print(f'  목표 높이로 (높이: {cfg.PICK_Z}cm)...')
        arm.move_to(robot_x, robot_y, cfg.PICK_Z)
        time.sleep(2.0)

        # 4. 홈 복귀
        print('  홈 복귀...')
        arm.home()
        time.sleep(3.0)

        print('완료!')

    except Exception as e:
        print(f'오류: {e}')
        arm.home()

    finally:
        is_running = False


# ── 시작 ─────────────────────────────────────────────────
arm.home()
time.sleep(3)

print('\nArUco 마커 + 파란 물체를 카메라에 비춰주세요')
print('SPACE: 접근  H: 홈  Q: 종료\n')

# ── 메인 루프 ────────────────────────────────────────────
while True:
    ret, frame = cap.read()
    if not ret:
        print('프레임 읽기 실패')
        break

    h, w = frame.shape[:2]

    # ArUco 감지
    px_per_cm, origin_px = detect_aruco(frame)

    # 물체 감지
    obj_px   = detect_object(frame)
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

        # main_vision.py 에서
        # 물체 좌표 표시 부분 수정

        if obj_px:
            rx, ry = pixel_to_robot(
                obj_px[0], obj_px[1],
                px_per_cm, origin_px
            )
            robot_xy = (rx, ry)

            # 물체 표시
            cv2.circle(frame, obj_px, 10, (0, 0, 255), -1)

            # 로봇팔 베이스 기준 좌표 표시
            cv2.putText(frame,
                f'Robot: ({rx}, {ry}) cm',
                (obj_px[0]+12, obj_px[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 255), 2)

            # 추가: 로봇팔 기준 방향 표시
            direction = ''
            if rx > 0:  direction += f'앞{rx}cm '
            if ry > 0:  direction += f'왼{ry}cm'
            if ry < 0:  direction += f'오른{abs(ry)}cm'

            cv2.putText(frame,
                direction,
                (obj_px[0]+12, obj_px[1]+25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 165, 0), 1)
            
            

    # 상태 메시지
    if is_running:
        msg   = 'Running... wait'
        color = (0, 165, 255)
    elif px_per_cm is None:
        msg   = 'ArUco marker needed'
        color = (0, 0, 255)
    elif obj_px is None:
        msg   = 'Object not found'
        color = (0, 165, 255)
    else:
        msg   = f'Object: ({robot_xy[0]}, {robot_xy[1]}) cm  SPACE: approach'
        color = (0, 255, 0)

    cv2.putText(frame, msg,
        (10, 65),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65, color, 2)

    cv2.putText(frame,
        'SPACE: approach  H: home  Q: quit',
        (10, h-15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (200, 200, 200), 1)

    cv2.imshow('Robot Vision', frame)

    # 키 입력
    key = cv2.waitKey(30) & 0xFF

    if key == ord('q'):
        print('\n홈 이동 후 종료...')
        arm.home()
        time.sleep(3.0)
        break

    elif key == ord('h'):
        if not is_running:
            print('홈 이동')
            arm.home()

    elif key == ord(' '):
        if is_running:
            print('동작 중 - 잠시 기다려주세요')
        elif robot_xy is None:
            print('물체 또는 마커 없음')
        else:
            t = threading.Thread(
                target=pick_and_place,
                args=(robot_xy[0], robot_xy[1])
            )
            t.daemon = True
            t.start()

# ── 종료 ─────────────────────────────────────────────────
cap.release()
cv2.destroyAllWindows()
arm.close()
print('종료')
