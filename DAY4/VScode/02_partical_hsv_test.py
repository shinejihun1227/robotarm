# partical_hsv_test.py
# 강의때는 파란색 물체 HSV 범위 확인 및 테스트

import cv2
import numpy as np

URL = 'http://192.0.0.2:8081/video'

# 파란색 기본 HSV 범위
lower = np.array([94, 63, 42])
upper = np.array([124, 255, 255])

cap = cv2.VideoCapture(URL)
print('파란색 인식 테스트')
print('파란 물체를 카메라 앞에 놓으세요')
print('c: 클릭 모드  q: 종료')

clicked_hsv = None
mode = 'auto'   # auto = 기본 파란색 / click = 클릭 모드

def on_mouse(event, x, y, flags, param):
    global clicked_hsv, lower, upper
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_frame  = cv2.cvtColor(param['frame'], cv2.COLOR_BGR2HSV)
        h, s, v    = hsv_frame[y, x]
        clicked_hsv = (int(h), int(s), int(v))

        # 클릭한 색상 기준으로 범위 자동 계산
        lower = np.array([max(0,   h-15), max(0,  s-50), max(0,  v-50)])
        upper = np.array([min(179, h+15), 255,           255           ])

        print(f'클릭 HSV: H={h} S={s} V={v}')
        print(f'  COLOR_LOWER = ({lower[0]}, {lower[1]}, {lower[2]})')
        print(f'  COLOR_UPPER = ({upper[0]}, {upper[1]}, {upper[2]})')

cv2.namedWindow('Blue Detection')
frame_data = {'frame': None}

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_data['frame'] = frame.copy()
    cv2.setMouseCallback('Blue Detection', on_mouse, frame_data)

    # HSV 마스크 생성
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    # 노이즈 제거
    kernel = np.ones((5, 5), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 컨투어 검출
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    obj_detected = False
    if contours:
        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)

        if area > 500:
            # 중심 계산
            M  = cv2.moments(largest)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # 원본에 표시
                cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                cv2.putText(frame,
                    f'물체: ({cx}, {cy})  면적: {int(area)}',
                    (cx+10, cy-10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2)
                obj_detected = True

    # 상태 표시
    status = '물체 감지됨!' if obj_detected else '물체를 비춰주세요'
    color  = (0, 255, 0)  if obj_detected else (0, 0, 255)
    cv2.putText(frame, status,
        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
        0.7, color, 2)

    cv2.putText(frame,
        f'Lower:{lower}',
        (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (255, 255, 0), 1)
    cv2.putText(frame,
        f'Upper:{upper}',
        (10, 85), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (255, 255, 0), 1)
    cv2.putText(frame,
        '물체 클릭 → 범위 자동 조정  q: 종료',
        (10, frame.shape[0]-10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (200, 200, 200), 1)

    # 마스크 오른쪽에 표시
    mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    combined   = np.hstack([frame, mask_color])

    cv2.imshow('Blue Detection', combined)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
