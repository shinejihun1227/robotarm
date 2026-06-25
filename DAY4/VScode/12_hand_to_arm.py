import cv2
import mediapipe as mp
import serial
import time
import math

PORT = 'COM7'
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print("✅ 로봇팔 다이렉트 제어 모드 접속 성공!")
    time.sleep(2)
except:
    ser = None
    print("❌ 로봇팔 접속 실패! 화면 테스트만 진행합니다.")

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
cap = cv2.VideoCapture(0)

prev_angles = [90, 90, 90, 90, 90, 90]
alpha = 0.4 

def map_range(x, in_min, in_max, out_min, out_max):
    return max(min(out_max, out_min), min(max(out_max, out_min), (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min))

last_send_time = time.time()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    res = hands.process(rgb)

    if res.multi_hand_landmarks:
        hand = res.multi_hand_landmarks[0]
        mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

        wrist = hand.landmark[0]
        mid_mcp = hand.landmark[9]  
        thumb_tip = hand.landmark[4]
        idx_tip = hand.landmark[8]

        # 1. 어깨 (가까움/머멂)
        palm_size = math.hypot(wrist.x - mid_mcp.x, wrist.y - mid_mcp.y)
        target_shoulder = map_range(palm_size, 0.1, 0.3, 70, 130)

        # 2. 팔꿈치 (위/아래)
        target_elbow = map_range(wrist.y, 0.2, 0.8, 140, 40)

        # ---------------------------------------------------------
        # 3. 베이스 & 손목 좌우 회전 (🔥방향 반전 & 민감도 폭발!🔥)
        # ---------------------------------------------------------
        dx = mid_mcp.x - wrist.x
        dy = mid_mcp.y - wrist.y
        angle_deg = math.degrees(math.atan2(dy, dx)) 
        
        # [수정됨] 
        # 입력 범위: -115 ~ -65 (겨우 50도만 까딱해도 다 돌아감. 더 민감하게 하려면 이 두 숫자의 차이를 줄이면 돼!)
        # 출력 범위: 180, 0 (이전의 0, 180을 뒤집어서 도는 방향을 반대로 바꿈!)
        roll_angle = map_range(angle_deg, -115, -65, 180, 0) 
        target_base = roll_angle
        target_wrist_r = roll_angle

        # 4. 손목 상하 (앞/뒤 젖힘)
        target_wrist_p = map_range(mid_mcp.z, -0.05, 0.05, 40, 140)

        # 5. 그리퍼 (열고 닫기)
        pinch_dist = math.hypot(thumb_tip.x - idx_tip.x, thumb_tip.y - idx_tip.y)
        target_grip = map_range(pinch_dist, 0.02, 0.1, 150, 40) 

        # 필터 적용 (스무딩)
        raw_targets = [target_base, target_shoulder, target_elbow, target_wrist_r, target_wrist_p, target_grip]
        for i in range(6):
            prev_angles[i] = prev_angles[i] * (1 - alpha) + raw_targets[i] * alpha

        # 데이터 전송
        if time.time() - last_send_time > 0.03:
            send_data = "A:" + ",".join([f"{int(a)}" for a in prev_angles]) + "\n"
            if ser: ser.write(send_data.encode())
            print(f"📡 전송 중: {send_data.strip()}") 
            last_send_time = time.time()
            
    else:
        # 손 상실 시 90도 복귀
        if time.time() - last_send_time > 0.1:
            send_data = "A:90,90,90,90,90,90\n"
            if ser: ser.write(send_data.encode())
            print("🖐️ 손 상실! - 정렬 상태(90도) 복귀 명령 쐈음!") 
            last_send_time = time.time()
            prev_angles = [90, 90, 90, 90, 90, 90] 

    cv2.imshow('Avatar Robot Control', frame)
    if cv2.waitKey(1) == ord('q'): break

if ser: ser.close()
cap.release()
cv2.destroyAllWindows()
  
