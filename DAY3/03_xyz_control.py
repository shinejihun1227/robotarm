# 03_xyz_control.py
from machine import I2C, Pin
from utime import sleep_ms
import config
from pca9685 import PCA9685
import ik

i2c = I2C(0, scl=Pin(config.I2C_SCL),
              sda=Pin(config.I2C_SDA), freq=400_000)
pca = PCA9685(i2c, config.PCA_ADDR)

_current = {s['name']: 0 for s in config.SERVO}
HOME_POSE = {
    'base': 0,
    'shoulder': 0,
    'elbow': 0,
    'wrist_r': 0,
    'wrist_p': 0,
    'grip': 0,
}

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _write(name, deg):
    idx     = {s['name']: i for i, s in enumerate(config.SERVO)}[name]
    cfg     = config.SERVO[idx]
    abs_deg = 90 + cfg['dir'] * deg + cfg['offset']
    abs_deg = _clamp(abs_deg, cfg['min_deg'], cfg['max_deg'])
    us      = int(cfg['min_us'] +
                  (cfg['max_us'] - cfg['min_us']) * abs_deg / 180)
    pca.set_us(config.SERVO_CH[idx], us)
    _current[name] = deg

def write_all(pose: dict):
    for name, deg in pose.items():
        _write(name, deg)

def move_smooth(pose: dict, step=1, delay_ms=30):
    """
    step=1  : 1도씩 이동 (부드럽게)
    delay=30: 30ms 간격 (천천히)
    """
    starts = {k: _current.get(k, 0) for k in pose}
    diffs  = {k: pose[k] - starts[k] for k in pose}
    ticks  = max((abs(int(d / step)) for d in diffs.values()), default=1)
    ticks  = max(ticks, 1)

    for i in range(ticks + 1):
        ratio = i / ticks
        frame = {k: starts[k] + diffs[k] * ratio for k in pose}
        write_all(frame)
        sleep_ms(delay_ms)

def move_to(x, y, z,
            wrist_r=0, wrist_p=0, grip=None,
            elbow_up=True):
    """
    (x, y, z) cm 위치로 천천히 이동
    """
    result = ik.ik(x, y, z, elbow_up=elbow_up)

    if result is None:
        print(f'  [실패] ({x}, {y}, {z}) 도달 불가')
        return False

    base, shoulder, elbow = result

    pose = {
        'base'    : base,
        'shoulder': shoulder,
        'elbow'   : elbow,
        'wrist_r' : wrist_r,
        'wrist_p' : wrist_p,
    }
    if grip is not None:
        pose['grip'] = grip

    move_smooth(pose)

    fx, fy, fz = ik.fk(base, shoulder, elbow)
    print(f'  목표: ({x}, {y}, {z})')
    print(f'  실제: ({fx}, {fy}, {fz})')
    print(f'  각도: base={base} sh={shoulder} el={elbow}')
    return True

def home():
    # shoulder=0 은 어깨 링크가 수직인 자세다.
    # elbow=-90 으로 두면 손목/그리퍼 쪽 링크가 앞으로 펴진 홈 자세가 된다.
    move_smooth(HOME_POSE)
    print('  홈 완료')

def where():
    b = _current.get('base',     0)
    s = _current.get('shoulder', 0)
    e = _current.get('elbow',    0)
    x, y, z = ik.fk(b, s, e)
    print(f'  현재: ({x}, {y}, {z}) cm')
    return x, y, z

# ── 실행 ────────────────────────────────────────────────
print('=== XYZ 제어 시작 ===')
home()
sleep_ms(2000)   # 홈 도착 후 2초 대기

print('\n테스트 1: 후진 이동')
#move_to(15, -10, 18)
home()
sleep_ms(2000)

print('\n홈 복귀')
home()

