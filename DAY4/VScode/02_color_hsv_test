# color_hsv_test.py
# 마우스로 클릭한 픽셀의 HSV 값 확인
# 실제 물체 색상 범위 찾는 용도

import cv2
import numpy as np

URL = 'http://192.0.0.2:8081/video'

cap = cv2.VideoCapture(URL)
print('HSV 색상 테스트')
print('물체를 클릭하면 HSV 값 출력')
print('q: 종료')

clicked_hsv = None

def on_mouse(event, x, y, flags, param):
    global clicked_hsv
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_frame = cv2.cvtColor(param['frame'], cv2.COLOR_BGR2HSV)
        h, s, v   = hsv_frame[y, x]
        clicked_hsv = (int(h), int(s), int(v))
        print(f'클릭 HSV: H={h} S={s} V={v}')
        print(f'  → COLOR_LOWER = ({max(0,h-15)}, {max(0,s-40)}, {max(0,v-40)})')
        print(f'  → COLOR_UPPER = ({min(179,h+15)}, 255, 255)')

cv2.namedWindow('HSV Test')
frame_data = {'frame': None}

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_data['frame'] = frame.copy()

    # 마우스 콜백 등록
    cv2.setMouseCallback('HSV Test',
        on_mouse, frame_data)

    # 클릭한 색상 마스크 표시
    if clicked_hsv:
        h, s, v = clicked_hsv
        lower = np.array([max(0,h-15), max(0,s-40), max(0,v-40)])
        upper = np.array([min(179,h+15), 255, 255])
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask  = cv2.inRange(hsv, lower, upper)

        # 마스크 오른쪽에 표시
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined   = np.hstack([frame, mask_color])
        cv2.putText(combined,
            f'HSV: {clicked_hsv}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 255, 0), 2)
        cv2.putText(combined,
            f'Lower: ({max(0,h-15)},{max(0,s-40)},{max(0,v-40)})',
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 0), 2)
        cv2.putText(combined,
            f'Upper: ({min(179,h+15)},255,255)',
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 0), 2)
        cv2.imshow('HSV Test', combined)
    else:
        cv2.putText(frame,
            '물체를 클릭하세요',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 255, 0), 2)
        cv2.imshow('HSV Test', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
