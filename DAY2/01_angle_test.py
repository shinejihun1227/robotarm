# 01_angle_test.py
from machine import I2C, Pin
from pca9685 import PCA9685
from utime import sleep_ms
import config

i2c = I2C(0, scl=Pin(config.I2C_SCL),
              sda=Pin(config.I2C_SDA), freq=400_000)
pca = PCA9685(i2c, config.PCA_ADDR)

NAME_TO_CH = {s['name']: i for i, s in enumerate(config.SERVO)}

# ── 서보 출력 ────────────────────────────────────────────
# dir, offset 적용된 _write 함수
def _write(name, deg):
    deg = max(0, min(180, deg))
    idx = NAME_TO_CH[name]
    cfg = config.SERVO[idx]

    # dir, offset 적용
    abs_deg = 90 + cfg['dir'] * (deg - 90) + cfg['offset']
    abs_deg = max(0, min(180, abs_deg))

    us = int(cfg['min_us'] +
             (cfg['max_us'] - cfg['min_us']) * abs_deg / 180)
    pca.set_us(config.SERVO_CH[idx], us)
    return us

# ── 90도 복귀 ────────────────────────────────────────────
def _return_to_neutral(name, current, delay):
    print('90도로 복귀 중...')
    r_step = 1 if current < 90 else -1
    deg = current
    while deg != 90:
        deg += r_step
        deg  = max(0, min(180, deg))
        _write(name, deg)
        sleep_ms(delay)
    print('복귀 완료 (90도)')

# ── 핵심 함수 ────────────────────────────────────────────
def move(name, direction, step=1, delay=30):
    """
    서보를 90도에서 천천히 이동
    Ctrl+C 누르면:
      → 즉시 멈춤
      → 현재 각도/us 출력
      → 천천히 90도 복귀

    name      : 서보 이름
    direction : 'up'   → 90도에서 180도로
                'down' → 90도에서 0도로
    step      : 한 번에 이동할 각도 (기본 1도)
    delay     : 각 스텝 사이 대기 ms (기본 30ms)

    사용 예:
      move('base', 'up')
      move('base', 'down')
      move('shoulder', 'up', 2)
    """
    if name not in NAME_TO_CH:
        print(f'이름 오류: {name}')
        print(f'사용 가능: {list(NAME_TO_CH.keys())}')
        return

    if direction not in ('up', 'down'):
        print("방향은 'up' 또는 'down' 만 가능해요")
        return

    # 시작은 항상 90도
    _write(name, 90)
    sleep_ms(500)

    end_deg = 180 if direction == 'up' else 0
    step    = step if direction == 'up' else -step

    print(f'{name} {direction} 시작 (Ctrl+C 누르면 정지)')

    deg = 90
    try:
        while True:
            # 끝 도달
            if (direction == 'up'   and deg >= end_deg) or \
               (direction == 'down' and deg <= end_deg):
                us = _write(name, deg)
                print(f'─────────────────────')
                print(f'끝 도달')
                print(f'  각도 : {deg}도')
                print(f'  펄스폭: {us}us')
                print(f'─────────────────────')
                _return_to_neutral(name, deg, delay)
                return deg, us

            # 이동
            deg += step
            deg  = max(0, min(180, deg))
            _write(name, deg)
            sleep_ms(delay)

    except KeyboardInterrupt:
        # Ctrl+C 눌렸을 때
        us = _write(name, deg)
        print(f'─────────────────────')
        print(f'정지!')
        print(f'  각도 : {deg}도')
        print(f'  펄스폭: {us}us')
        print(f'─────────────────────')
        _return_to_neutral(name, deg, delay)
        return deg, us


# ── 전체 90도 ────────────────────────────────────────────
def all_neutral():
    """전체 서보 90도 복귀"""
    for name in NAME_TO_CH:
        _write(name, 90)
    print('전체 90도 완료')


# ── 설정값 확인 ──────────────────────────────────────────
def status():
    """config.py 설정값 출력"""
    print('\n현재 config.py 설정:')
    print(f'  {"채널":<4} {"이름":<10} {"방향":<6} {"오프셋":<8} {"범위":<12}')
    print(f'  {"─"*44}')
    for i, s in enumerate(config.SERVO):
        print(f'  CH{i}  {s["name"]:10s} '
              f'dir={s["dir"]:+d}  '
              f'offset={s["offset"]:+3d}  '
              f'{s["min_deg"]}~{s["max_deg"]}도')
    print()


# ── 시작 ────────────────────────────────────────────────
all_neutral()
sleep_ms(500)
print()
print('=' * 40)
print('사용법:')
print()
print("  move('base', 'up')       # 90→180도")
print("  move('base', 'down')     # 90→0도")
print("  Ctrl+C                   # 정지 + 값 출력 + 복귀")
print()
print("  all_neutral()            # 전체 90도 복귀")
print("  status()                 # 설정값 확인")
print('=' * 40)                                    
