# aruco_test.py
import cv2
import numpy as np

URL = 'http://192.0.0.2:8081/video'

# ArUco 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params     = cv2.aruco.DetectorParameters()
detector   = cv2.aruco.ArucoDetector(aruco_dict, params)

MARKER_CM  = 4.0   # 실제 마커 크기

cap = cv2.VideoCapture(URL)
print('ArUco 테스트 시작')
print('마커를 카메라에 비춰보세요')
print('q: 종료')

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        # 마커 표시
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i, mid in enumerate(ids.flatten()):
            c  = corners[i][0]
            cx = int(c[:, 0].mean())
            cy = int(c[:, 1].mean())

            # 픽셀당 cm 계산
            side_px   = np.linalg.norm(c[0] - c[1])
            px_per_cm = side_px / MARKER_CM

            # 마커 정보 표시
            cv2.putText(frame,
                f'ID:{mid} ({cx},{cy})',
                (cx-40, cy-15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)

            # 마커 0번이면 스케일 표시
            if mid == 0:
                cv2.putText(frame,
                    f'Scale: {px_per_cm:.1f} px/cm',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 0), 2)

        print(f'감지된 마커: {ids.flatten().tolist()}')

    else:
        cv2.putText(frame,
            '마커를 비춰주세요',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 0, 255), 2)

    cv2.imshow('ArUco Test', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
