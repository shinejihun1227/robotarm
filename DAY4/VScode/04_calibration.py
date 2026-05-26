# calibration.py
import cv2
import numpy as np
import json
import re
import config_vision as cfg

URL = cfg.CAMERA_URL

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params     = cv2.aruco.DetectorParameters()
detector   = cv2.aruco.ArucoDetector(aruco_dict, params)

def detect_aruco(frame):
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

def detect_object(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(cfg.COLOR_LOWER),
        np.array(cfg.COLOR_UPPER)
    )
    kernel = np.ones((5,5), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
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
    return (int(M['m10']/M['m00']), int(M['m01']/M['m00']))

def save_config(offset_x, offset_y):
    """
    config_vision.py 에서
    BASE_OFFSET_X, BASE_OFFSET_Y 값을
    정규식으로 찾아서 교체
    """
    with open('config_vision.py', 'r', encoding='utf-8') as f:
        content = f.read()

    # 정규식으로 숫자 부분만 교체
    # BASE_OFFSET_X = 숫자 형태를 찾아서 교체
    content = re.sub(
        r'BASE_OFFSET_X\s*=\s*[-\d.]+',
        f'BASE_OFFSET_X = {round(offset_x, 1)}',
        content
    )
    content = re.sub(
        r'BASE_OFFSET_Y\s*=\s*[-\d.]+',
        f'BASE_OFFSET_Y = {round(offset_y, 1)}',
        content
    )

    with open('config_vision.py', 'w', encoding='utf-8') as f:
        f.write(content)

    # calibration.json 저장
    with open('calibration.json', 'w', encoding='utf-8') as f:
        json.dump({
            'BASE_OFFSET_X': round(offset_x, 1),
            'BASE_OFFSET_Y': round(offset_y, 1),
        }, f, indent=2)

    print(f'✅ config_vision.py 저장 완료')
    print(f'   BASE_OFFSET_X = {round(offset_x, 1)}')
    print(f'   BASE_OFFSET_Y = {round(offset_y, 1)}')


print('=' * 50)
print('캘리브레이션 시작')
print('=' * 50)
print()
print('순서:')
print('  1. 로봇팔 홈 위치로 이동')
print('  2. 그리퍼 끝 아래에 파란 물체 놓기')
print('  3. SPACE 3~5번 눌러서 측정')
print('  4. S 키로 저장')
print('  q: 종료')
print()

cap = cv2.VideoCapture(URL)
if not cap.isOpened():
    print('카메라 연결 실패')
    exit()

# FK 홈 위치 계산
import sys
sys.path.append('.')
try:
    import importlib.util
    # config_vision 에서 링크 길이 가져오기
    home_x = cfg.BASE_OFFSET_X   # 임시값 (측정으로 교체됨)
except:
    home_x = 20.0

offset_samples = []

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]

    px_per_cm, origin_px = detect_aruco(frame)
    obj_px = detect_object(frame)

    # 화면 표시
    if px_per_cm and origin_px:
        cv2.putText(frame,
            f'Scale: {px_per_cm:.1f} px/cm',
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255,255,0), 2)
        cv2.circle(frame, origin_px, 8, (0,255,0), -1)
        cv2.putText(frame, 'Marker0',
            (origin_px[0]+10, origin_px[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0,255,0), 1)

        if obj_px:
            cv2.circle(frame, obj_px, 10, (0,0,255), -1)
            dx_cm = (obj_px[0] - origin_px[0]) / px_per_cm
            dy_cm = (obj_px[1] - origin_px[1]) / px_per_cm
            cv2.putText(frame,
                f'marker 기준: dx={dx_cm:.1f} dy={dy_cm:.1f}',
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0,0,255), 2)

    # 측정 횟수 표시
    if offset_samples:
        avg_x = sum(s[0] for s in offset_samples)/len(offset_samples)
        avg_y = sum(s[1] for s in offset_samples)/len(offset_samples)
        cv2.putText(frame,
            f'측정 {len(offset_samples)}회 | '
            f'OFFSET_X={avg_x:.1f} OFFSET_Y={avg_y:.1f}',
            (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0,255,0), 2)

    status = 'ArUco 마커 필요' if not px_per_cm \
        else '파란 물체 필요' if not obj_px \
        else 'SPACE: 측정  S: 저장  Q: 종료'
    cv2.putText(frame, status,
        (10, h-15), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (200,200,200), 1)

    cv2.imshow('Calibration', frame)
    key = cv2.waitKey(30) & 0xFF

    if key == ord('q'):
        break

    elif key == ord(' '):
        if not px_per_cm or not obj_px:
            print('마커 또는 물체를 찾지 못했어요')
            continue

        dx_cm = (obj_px[0] - origin_px[0]) / px_per_cm
        dy_cm = (obj_px[1] - origin_px[1]) / px_per_cm

        # 오프셋 계산
        # 로봇 홈 x 위치 (FK 결과)
        robot_home_x = 19.5   # fk(0,0,0) 의 x 값
        robot_home_y = 0.0    # fk(0,0,0) 의 y 값

        offset_x = robot_home_x + dy_cm
        offset_y = robot_home_y - dx_cm

        offset_samples.append((offset_x, offset_y))
        print(f'측정 {len(offset_samples)}회: '
              f'OFFSET_X={offset_x:.1f}  OFFSET_Y={offset_y:.1f}')

    elif key == ord('s'):
        if not offset_samples:
            print('측정값 없음 - SPACE 먼저 눌러주세요')
            continue

        avg_x = sum(s[0] for s in offset_samples)/len(offset_samples)
        avg_y = sum(s[1] for s in offset_samples)/len(offset_samples)

        save_config(avg_x, avg_y)
        print(f'측정 횟수: {len(offset_samples)}회')
        break

cap.release()
cv2.destroyAllWindows()
print('캘리브레이션 완료')
