import cv2

# 수정된 URL (반드시 본인 핸드폰 주소로 확인!)
url = 'http://192.0.0.2:8081/video' 

# 스트리밍 캡처 객체 생성
cap = cv2.VideoCapture(url)

print("카메라 연결 중...")

while True:
    # 한 프레임씩 읽어오기
    ret, frame = cap.read()

    # 읽기 실패 시 루프 탈출
    if not ret:
        print("프레임을 읽어올 수 없습니다. 주소나 연결을 확인하세요.")
        break

    # 화면에 출력
    cv2.imshow('Camera Test', frame)

    # 'q' 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 종료 처리
cap.release()
cv2.destroyAllWindows()
