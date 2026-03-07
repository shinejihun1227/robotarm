# id 0 과 id 1 간의 중심간 거리를 잴 수 있음 : 둘 다 수평으로 뒀을 때 기준. 두 거리 간에 노란색 선 표시가 됨 cv.

import cv2
import cv2.aruco as aruco
import numpy as np

# 1. 설정값 반영 (cm 단위)
MARKER_REAL_SIZE = 5.0 
HALF_SIZE = MARKER_REAL_SIZE / 2.0  # 2.5cm

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

cap = cv2.VideoCapture(0)

print("--- Phase 2: 중앙 원점 기준 좌표 추적 모드 ---")

while True:
    ret, frame = cap.read()
    if not ret: break

    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        id_list = ids.flatten()

        # [기준점] ID 0번 마커를 원점으로 설정
        if 0 in id_list:
            idx0 = np.where(id_list == 0)[0][0]
            c0 = corners[idx0][0]

            # 실제 세상의 좌표: 중앙을 (0,0)으로 잡기 위해 범위를 -2.5 ~ 2.5로 설정
            pts_dst = np.array([
                [-HALF_SIZE, -HALF_SIZE], # Top-Left
                [ HALF_SIZE, -HALF_SIZE], # Top-Right
                [ HALF_SIZE,  HALF_SIZE], # Bottom-Right
                [-HALF_SIZE,  HALF_SIZE]  # Bottom-Left
            ], dtype=float)

            # 호모그래피 행렬 H 계산
            h, _ = cv2.findHomography(c0, pts_dst)
            
            # 원점(중앙) 표시
            c0_center = np.mean(c0, axis=0)
            cv2.circle(frame, (int(c0_center[0]), int(c0_center[1])), 5, (255, 0, 0), -1)
            cv2.putText(frame, "Origin (0,0)", (int(c0_center[0])-20, int(c0_center[1])-20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # [추적 대상] ID 1번 마커 추적
            if 1 in id_list:
                idx1 = np.where(id_list == 1)[0][0]
                c1_center = np.mean(corners[idx1][0], axis=0)
                
                # 호모그래피 변환 (실제 좌표 도출)
                point = np.array([c1_center[0], c1_center[1], 1]).reshape(3, 1)
                real_pos = np.dot(h, point)
                
                rx = real_pos[0][0] / real_pos[2][0]
                ry = real_pos[1][0] / real_pos[2][0]

                # 결과 출력 (ID 0의 중앙으로부터의 상대 거리)
                pos_text = f"X: {rx:.1f}cm, Y: {ry:.1f}cm"
                print(f"Target Pos -> {pos_text}")
                cv2.line(frame, (int(c0_center[0]), int(c0_center[1])), 
                         (int(c1_center[0]), int(c1_center[1])), (0, 255, 255), 2)
                cv2.putText(frame, pos_text, (int(c1_center[0]), int(c1_center[1])-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow('Dwaeji Robot Vision - Center Origin', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()
