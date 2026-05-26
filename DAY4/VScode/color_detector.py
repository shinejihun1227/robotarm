# color_detector.py
# 색상으로 물체 감지

import cv2
import numpy as np
import config_vision as cfg

class ColorDetector:

    def detect(self, frame):
        """
        프레임에서 색상 물체 감지

        반환:
          (cx, cy) 물체 중심 픽셀
          None     감지 실패
        """
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

        # 가장 큰 컨투어
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 500:
            return None

        M  = cv2.moments(largest)
        if M['m00'] == 0:
            return None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return cx, cy
