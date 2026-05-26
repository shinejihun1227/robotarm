# aruco_detector.py
# ArUco 마커 인식 + 좌표 변환

import cv2
import numpy as np
import config_vision as cfg

class ArucoDetector:

    def __init__(self):
        # ArUco 딕셔너리 설정
        aruco_dict = getattr(cv2.aruco, cfg.ARUCO_DICT)
        self.dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.params     = cv2.aruco.DetectorParameters()
        self.detector   = cv2.aruco.ArucoDetector(
            self.dictionary, self.params
        )
        self.camera_matrix = None
        self.dist_coeffs   = None

    def set_camera_params(self, camera_matrix, dist_coeffs):
        """카메라 캘리브레이션 파라미터 설정"""
        self.camera_matrix = camera_matrix
        self.dist_coeffs   = dist_coeffs

    def detect(self, frame):
        """
        프레임에서 ArUco 마커 감지

        반환:
          markers : {id: (cx, cy, corners)} 딕셔너리
          frame   : 마커 표시된 프레임
        """
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        markers = {}
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                c  = corners[i][0]
                cx = int(c[:, 0].mean())
                cy = int(c[:, 1].mean())
                markers[int(mid)] = (cx, cy, c)

            # 화면에 마커 표시
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        return markers, frame

    def get_scale(self, markers):
        """
        마커 크기로 픽셀당 cm 비율 계산

        마커 0번 기준으로 계산
        """
        if 0 not in markers:
            return None, None

        corners = markers[0][2]
        # 마커 한 변 픽셀 길이
        side_px = np.linalg.norm(corners[0] - corners[1])
        px_per_cm = side_px / cfg.MARKER_CM

        origin_px = (markers[0][0], markers[0][1])
        return px_per_cm, origin_px

    def pixel_to_robot(self, px, py, px_per_cm, origin_px):
        """
        픽셀 좌표 → 로봇팔 좌표계 (cm) 변환

        px, py      : 물체 픽셀 좌표
        px_per_cm   : 픽셀당 cm 비율
        origin_px   : 마커0 픽셀 좌표 (기준점)
        """
        # 마커 기준 상대 픽셀
        dx_px = px - origin_px[0]
        dy_px = py - origin_px[1]

        # 픽셀 → cm
        dx_cm = dx_px / px_per_cm
        dy_cm = dy_px / px_per_cm

        # 로봇팔 베이스 좌표계로 변환
        # 카메라가 위에서 아래로 찍는 경우
        robot_x = cfg.BASE_OFFSET_X - dy_cm
        #robot_y = -dx_cm
        robot_y = dx_cm


        return round(robot_x, 1), round(robot_y, 1)

    def draw_info(self, frame, px_per_cm, origin_px,
                  obj_px=None, robot_xy=None):
        """화면에 정보 표시"""
        h, w = frame.shape[:2]

        # 스케일 표시
        if px_per_cm:
            cv2.putText(frame,
                f'Scale: {px_per_cm:.1f} px/cm',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 0), 2)

        # 물체 위치 표시
        if obj_px and robot_xy:
            cv2.circle(frame, obj_px, 10, (0, 0, 255), -1)
            cv2.putText(frame,
                f'({robot_xy[0]:.1f}, {robot_xy[1]:.1f}) cm',
                (obj_px[0]+12, obj_px[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 255), 2)

        # 안내 메시지
        cv2.putText(frame,
            'SPACE: 집기  H: 홈  Q: 종료',
            (10, h-15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (200, 200, 200), 1)

        return frame
