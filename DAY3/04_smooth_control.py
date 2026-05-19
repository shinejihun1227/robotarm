# 04_smooth_control.py
# 사다리꼴 프로파일 적용한 부드러운 XYZ 제어
# Thonny 에서 직접 Run

from machine import I2C, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import config
from pca9685 import PCA9685
import ik
from motion import trapezoidal, sync_profiles

# ── 초기화 ──────────────────────────────────────────────
i2c = I2C(0, scl=Pin(config.I2C_SCL),
              sda=Pin(config.I2C_SDA), freq=400_000)
pca = PCA9685(i2c, config.PCA_ADDR)

# ── 속도 설정 ────────────────────────────────────────────
V_MAX = 120.0   # 최대 속도 (도/초)  ← 크면 빠름
A_MAX = 200.0   # 최대 가속도 (도/초²) ← 크면 딱딱함
DT    = 0.02    # 샘플 간격 20ms = 50Hz

# ── 현재 상태 ────────────────────────────────────────────
_current = {s['name']: 0 for s in config.SERVO}

# ── 서보 출력 ────────────────────────────────────────────
def _clamp(val, lo, hi):
    return max(lo, min(hi, val))

def write_servo(idx, deg):
    cfg     = config.SERVO[idx]
    abs_deg = 90 + cfg['dir'] * deg + cfg['offset']
    abs_deg = _clamp(abs_deg, cfg['min_deg'], cfg['max_deg'])
    us      = int(cfg['min_us'] +
                  (cfg['max_us'] - cfg['min_us']) * abs_deg / 180)
    pca.set_us(config.SERVO_CH[idx], us)

def write_all(pose: dict):
    name_to_idx = {s['name']: i for i, s in enumerate(config.SERVO)}
    for name, deg in pose.items():
        if name in name_to_idx:
            write_servo(name_to_idx[name], deg)


# ── 사다리꼴 이동 ─────────────────────────────────────────
def move_trap(target: dict, v_max=V_MAX, a_max=A_MAX):
    """
    사다리꼴 속도 프로파일로 부드럽게 이동

    모든 축이 동시에 출발해서 동시에 도착
    """
    global _current

    starts = {k: _current.get(k, 0) for k in target}

    # 프로파일 생성
    profiles = sync_profiles(starts, target, v_max, a_max, DT)

    n = len(next(iter(profiles.values())))

    # 실행
    for i in range(n):
        t0   = ticks_ms()
        frame = {k: profiles[k][i] for k in profiles}
        write_all(frame)

        # 정확한 타이밍 유지
        elapsed = ticks_diff(ticks_ms(), t0)
        wait    = int(DT * 1000) - elapsed
        if wait > 0:
            sleep_ms(wait)

    _current.update(target)


# ── XYZ 이동 ─────────────────────────────────────────────
def move_to(x, y, z,
            wrist_r=0, wrist_p=0, grip=None,
            elbow_up=True, v_max=V_MAX, a_max=A_MAX):
    """
    그리퍼를 (x, y, z) cm 위치로 부드럽게 이동

    v_max : 속도 조절 (작을수록 느림)
    a_max : 가속도 조절 (작을수록 부드러움)
    """
    result = ik.ik(x, y, z, elbow_up=elbow_up)

    if result is None:
        print(f'  [실패] ({x}, {y}, {z}) 도달 불가')
        return False

    base, shoulder, elbow = result

    target = {
        'base'    : base,
        'shoulder': shoulder,
        'elbow'   : elbow,
        'wrist_r' : wrist_r,
        'wrist_p' : wrist_p,
    }
    if grip is not None:
        target['grip'] = grip

    move_trap(target, v_max=v_max, a_max=a_max)

    fx, fy, fz = ik.fk(base, shoulder, elbow)
    print(f'  ({x},{y},{z}) → '
          f'base={base:.0f} sh={shoulder:.0f} el=	{elbow:.0f}  '
          f'실제=({fx},{fy},{fz})')
    return True

def home():
    move_trap({'base':0,'shoulder':0,'elbow':0,
               'wrist_r':0,'wrist_p':0,'grip':0})
    print('  홈')

def where():
    b = _current.get('base',     0)
    s = _current.get('shoulder', 0)
    e = _current.get('elbow',    0)
    x, y, z = ik.fk(b, s, e)
    print(f'  현재: ({x}, {y}, {z}) cm')
    return x, y, z


# ── 실행 ────────────────────────────────────────────────
print('=== 부드러운 이동 테스트 ===')

home()
sleep_ms(500)

# 속도 비교 테스트
print('\n빠른 이동 (v_max=150):')
move_to(20, 0, 15, v_max=150, a_max=300)
sleep_ms(300)

print('\n느린 이동 (v_max=50):')
move_to(15, 10, 15, v_max=50, a_max=100)
sleep_ms(300)

print('\n기본 이동:')
move_to(18, -10, 20)
sleep_ms(300)

where()
sleep_ms(300)
home()
