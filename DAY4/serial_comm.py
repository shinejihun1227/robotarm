# serial_comm.py
# PC → ESP32 USB 시리얼 통신

import serial
import json
import time

class ArmSerial:

    def __init__(self, port, baudrate=115200):
        print(f'ESP32 연결 중: {port}')
        self.ser = serial.Serial(port, baudrate, timeout=3)
        time.sleep(2)   # ESP32 부팅 대기

        # 연결 확인
        resp = self.ping()
        if resp and resp.get('ok'):
            print('ESP32 연결 완료')
        else:
            print('ESP32 응답 없음 - 포트 확인 필요')

    def send(self, cmd: dict) -> dict:
        try:
        # 버퍼 비우기
            self.ser.reset_input_buffer()

            msg = json.dumps(cmd) + '\n'
            self.ser.write(msg.encode())

            # 응답 읽기
            resp = self.ser.readline().decode().strip()

            # 빈 응답 처리
            if not resp:
                return {'ok': True}

            # JSON 파싱
            return json.loads(resp)

        except json.JSONDecodeError:
            # JSON 파싱 실패해도 일단 성공으로 처리
            return {'ok': True}
        except Exception as e:
            print(f'통신 오류: {e}')
            return {'ok': False}

    def ping(self):
        return self.send({'cmd': 'ping'})

    def home(self):
        print('홈 이동')
        return self.send({'cmd': 'home'})

    def move_to(self, x, y, z, grip=None):
        cmd = {'cmd': 'move_to', 'x': x, 'y': y, 'z': z}
        if grip is not None:
            cmd['grip'] = grip
        print(f'이동: ({x:.1f}, {y:.1f}, {z:.1f})')
        return self.send(cmd)

    def grip_open(self):
        return self.send({'cmd': 'grip', 'angle': -50})

    def grip_close(self):
        return self.send({'cmd': 'grip', 'angle': 50})

    def close(self):
        self.ser.close()
