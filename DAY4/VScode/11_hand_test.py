import cv2
import mediapipe as mp

# MediaPipe 손 인식 모델 초기화
print("내가 찾은 미디어파이프 위치:", mp.__file__)

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    max_num_hands=1,               # 인식할 손의 최대 개수 (일단 1개만)
    min_detection_confidence=0.5,  # 손을 찾는 최소 신뢰도 (0.5=50% 이상 확신할 때만 손으로 인정)
    min_tracking_confidence=0.5    # 손을 추적하는 최소 신뢰도
)

# 노트북 내장 웹캠 켜기 (0번 카메라)
# 만약 핸드폰 카메라를 쓰고 싶다면 0 대신 'http://...' 주소를 넣으면 돼!
cap = cv2.VideoCapture(0)

print("카메라를 켭니다. 종료하려면 카메라 창을 누르고 'q'를 누르세요.")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("비디오 프레임을 읽을 수 없습니다.")
        break

    # 화면 좌우 반전 (거울처럼 보이기 위해)
    frame = cv2.flip(frame, 1)

    # OpenCV는 BGR 색상을 쓰고, MediaPipe는 RGB 색상을 써서 변환해줘야 해
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # MediaPipe로 손 찾기 진행!
    result = hands.process(rgb_frame)

    # 손이 화면에 인식되었다면?
    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            # 인식된 손의 관절(랜드마크)에 점과 선을 그려줘
            mp_drawing.draw_landmarks(
                frame, 
                hand_landmarks, 
                mp_hands.HAND_CONNECTIONS
            )

            # (보너스) 8번 랜드마크 = 검지 손가락 끝부분의 화면상 좌표(비율) 출력
            index_finger_tip = hand_landmarks.landmark[8]
            print(f"검지 손가락 끝 좌표 -> X: {index_finger_tip.x:.2f}, Y: {index_finger_tip.y:.2f}")

    # 최종 결과 화면에 보여주기
    cv2.imshow('Hand Tracking (Deji)', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 끝날 때 깔끔하게 정리
cap.release()
cv2.destroyAllWindows()
