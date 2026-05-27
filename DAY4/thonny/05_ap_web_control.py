# 05_ap_web_control.py
# ESP32 AP Web Control 

import network
import socket
import json
from time import sleep_ms, ticks_ms, ticks_diff

# ══════════════════════════════════════════════════════════
# 1. WiFi AP
# ══════════════════════════════════════════════════════════
SSID        = 'RobotArm'
PASSWORD    = '12345678'
MAX_CLIENTS = 4

ap = network.WLAN(network.AP_IF)
ap.active(True)

try:
    ap.config(
        essid=SSID,
        password=PASSWORD,
        authmode=3,
        max_clients=MAX_CLIENTS
    )
except TypeError:
    ap.config(essid=SSID, password=PASSWORD, authmode=3)
    try:
        ap.config(max_clients=MAX_CLIENTS)
    except Exception:
        print('max_clients 옵션 미지원 - 기본값으로 동작')

while not ap.active():
    sleep_ms(100)

print('=' * 45)
print('  AP 시작:', SSID)
print('  접속 제한:', MAX_CLIENTS, '대')
print('  IP:', ap.ifconfig()[0])
print('  브라우저: http://192.168.4.1')
print('=' * 45)

# ══════════════════════════════════════════════════════════
# 2. Hardware
# ══════════════════════════════════════════════════════════
from machine import I2C, Pin
from pca9685 import PCA9685
import config
import ik
from motion import sync_profiles

try:
    i2c = I2C(0, scl=Pin(config.I2C_SCL),
                  sda=Pin(config.I2C_SDA), freq=400_000)
    pca = PCA9685(i2c, config.PCA_ADDR)
    HW_OK = True
    print('하드웨어 초기화 성공')
except Exception as e:
    HW_OK = False
    print('하드웨어 오류:', e)
    print('웹서버는 동작하지만 로봇팔은 움직이지 않습니다')

# ══════════════════════════════════════════════════════════
# 3. Settings
# ══════════════════════════════════════════════════════════
DEFAULT_V_MAX = 60.0
DEFAULT_A_MAX = 120.0
DT            = 0.02

# angle scan 설정
SCAN_STEP_DEG      = 1.0     # 작을수록 부드럽지만 느림
SCAN_INTERVAL_MS   = 25      # 작을수록 부드럽지만 ESP32 부담 증가
SAFE_MARGIN_DEG    = 5.0     # min/max 추천값에 적용할 안전 여유

# 원본 00_neutral.py 방식: offset을 고려하지 않는 중립 펄스폭
RAW_NEUTRAL_US = {
    0: 1500,   # base
    1: 1500,   # shoulder
    2: 1500,   # elbow
    3: 1450,   # wrist_r
    4: 1450,   # wrist_p
    5: 1450,   # grip
}

NAME_TO_IDX = {s['name']: i for i, s in enumerate(config.SERVO)}

# relative angle 기준 현재 상태
# rel=0이면 보정된 중립이며 실제 출력은 90 + offset
_current = {s['name']: 0.0 for s in config.SERVO}

# calibration absolute 상태 표시용
_calib_abs = {s['name']: 90.0 for s in config.SERVO}

# angle scan 상태
_scan = {
    'active': False,
    'name': None,
    'direction': None,
    'last_ms': 0,
    'last_msg': 'scan 대기',
}


# ══════════════════════════════════════════════════════════
# 4. Utility
# ══════════════════════════════════════════════════════════
def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def abs_to_us(idx, abs_deg):
    cfg = config.SERVO[idx]
    return int(cfg['min_us'] +
               (cfg['max_us'] - cfg['min_us']) * abs_deg / 180)


def rel_to_abs(idx, rel_deg):
    """
    rel_deg:
      0 = calibrated neutral
      + = one side
      - = opposite side

    abs_deg:
      actual servo command angle after dir / offset
    """
    cfg = config.SERVO[idx]
    return 90 + cfg['dir'] * rel_deg + cfg['offset']


def abs_to_rel(idx, abs_deg):
    cfg = config.SERVO[idx]
    return (abs_deg - 90 - cfg['offset']) / cfg['dir']


def allowed_rel_range(idx):
    """
    config.py의 min_deg/max_deg는 absolute servo angle 기준.
    웹 조그/IK는 relative angle을 쓰므로, 상대각 허용 범위를 계산한다.
    """
    cfg = config.SERVO[idx]
    r1 = (cfg['min_deg'] - 90 - cfg['offset']) / cfg['dir']
    r2 = (cfg['max_deg'] - 90 - cfg['offset']) / cfg['dir']
    return min(r1, r2), max(r1, r2)


def check_joint(idx, rel_deg):
    cfg = config.SERVO[idx]
    abs_deg = rel_to_abs(idx, rel_deg)

    if abs_deg < cfg['min_deg'] or abs_deg > cfg['max_deg']:
        rmin, rmax = allowed_rel_range(idx)
        msg = (
            '{} 범위 초과 | rel={:.1f}, abs={:.1f}, '
            'allowed_abs={}~{}, allowed_rel={:.1f}~{:.1f}'
        ).format(cfg['name'], rel_deg, abs_deg,
                 cfg['min_deg'], cfg['max_deg'], rmin, rmax)
        return False, msg

    return True, 'ok'


def check_pose(pose):
    for name, rel_deg in pose.items():
        if name not in NAME_TO_IDX:
            return False, '알 수 없는 관절: {}'.format(name)
        ok, msg = check_joint(NAME_TO_IDX[name], rel_deg)
        if not ok:
            return False, msg
    return True, 'ok'


def write_servo_rel(idx, rel_deg):
    """
    일반 제어용 출력.
    offset, dir을 반영한다.
    """
    if not HW_OK:
        return

    cfg = config.SERVO[idx]
    abs_deg = rel_to_abs(idx, rel_deg)
    abs_deg = _clamp(abs_deg, cfg['min_deg'], cfg['max_deg'])
    us = abs_to_us(idx, abs_deg)
    pca.set_us(config.SERVO_CH[idx], us)


def write_servo_abs(idx, abs_deg):
    """
    offset calibration / raw neutral용 출력.
    offset, dir을 반영하지 않는다.
    """
    if not HW_OK:
        return

    cfg = config.SERVO[idx]
    abs_deg = _clamp(abs_deg, 0, 180)
    us = abs_to_us(idx, abs_deg)
    pca.set_us(config.SERVO_CH[idx], us)


def write_all_rel(pose):
    for name, rel_deg in pose.items():
        if name in NAME_TO_IDX:
            write_servo_rel(NAME_TO_IDX[name], rel_deg)


def move_one_joint_to_zero(name):
    if name in NAME_TO_IDX:
        move_trap({name: 0.0}, DEFAULT_V_MAX, DEFAULT_A_MAX)


# ══════════════════════════════════════════════════════════
# 5. 00 Neutral / Home
# ══════════════════════════════════════════════════════════
def raw_neutral():
    """
    원본 00_neutral.py 개념.
    offset을 고려하지 않고 고정 펄스폭을 출력한다.
    """
    if not HW_OK:
        return False, '하드웨어 미연결'

    for idx, cfg in enumerate(config.SERVO):
        us = RAW_NEUTRAL_US.get(idx, abs_to_us(idx, 90))
        pca.set_us(config.SERVO_CH[idx], us)

        _calib_abs[cfg['name']] = 90.0
        _current[cfg['name']] = 0.0

    return True, '00 Raw Neutral 완료 | offset 미적용'


def calibrated_home(use_smooth=True):
    """
    보정된 중립 위치.
    rel=0이므로 실제 출력은 abs = 90 + offset.
    """
    target = {s['name']: 0.0 for s in config.SERVO}
    if use_smooth:
        ok, msg = move_trap(target, DEFAULT_V_MAX, DEFAULT_A_MAX)
    else:
        ok, msg = move_linear(target)
    if ok:
        return True, 'Calibrated Home 완료 | offset 적용'
    return False, msg


# ══════════════════════════════════════════════════════════
# 6. 01 Offset Calibration
# ══════════════════════════════════════════════════════════
def calib_set_joint(name, abs_deg):
    """
    offset calibration용.
    grip을 제외한 일반 관절은 absolute servo command angle 기준으로 움직인다.

    실제 하드웨어 기준 자세가 맞았을 때:
      recommended offset = abs_deg - 90
    """
    global _current

    if name not in NAME_TO_IDX:
        return False, '관절 이름 오류: {}'.format(name)

    if name == 'grip':
        return False, 'grip은 rel 기준 calibration을 사용하세요'

    idx = NAME_TO_IDX[name]
    cfg = config.SERVO[idx]
    abs_deg = float(abs_deg)

    # calibration 단계도 안전하게 config min/max 안으로 제한
    abs_deg = _clamp(abs_deg, cfg['min_deg'], cfg['max_deg'])

    write_servo_abs(idx, abs_deg)

    _calib_abs[name] = abs_deg
    _current[name] = abs_to_rel(idx, abs_deg)

    offset = abs_deg - 90
    msg = (
        '{} calibration | software_abs={:.1f}deg | '
        'error={:+.1f}deg | recommended offset={:+.1f}'
    ).format(name, abs_deg, offset, offset)

    return True, msg


def calib_set_grip_rel(rel_deg):
    """
    grip 전용 calibration.
    grip은 offset보다 열림/닫힘 관계가 중요하므로 rel 기준으로 표시한다.

    config.py에서 grip dir=-1, offset=0 기준:
      rel=0  -> abs=90 -> 닫힘
      rel=30 -> abs=60 -> 열림
    """
    global _current

    name = 'grip'
    if name not in NAME_TO_IDX:
        return False, 'grip 관절이 config.SERVO에 없습니다'

    idx = NAME_TO_IDX[name]
    rel_deg = float(rel_deg)

    ok, msg = check_joint(idx, rel_deg)
    if not ok:
        return False, msg

    write_servo_rel(idx, rel_deg)
    _current[name] = rel_deg

    abs_deg = rel_to_abs(idx, rel_deg)

    if rel_deg <= 0:
        state = '닫힘 기준'
    elif rel_deg < 30:
        state = '부분 열림'
    else:
        state = '열림 기준'

    msg = (
        'grip rel calibration | rel={:.1f}deg | abs={:.1f}deg | {}\n'
        '설명: 웹 rel 값이 커질수록 열리게 하려면 config.py에서 grip dir=-1 권장'
    ).format(rel_deg, abs_deg, state)

    return True, msg


# ══════════════════════════════════════════════════════════
# 7. 01 Angle Test
# ══════════════════════════════════════════════════════════
def jog_joint(name, delta, smooth=True):
    if name not in NAME_TO_IDX:
        return False, '관절 이름 오류: {}'.format(name)

    scan_stop_internal(return_zero=False)

    target_val = _current.get(name, 0.0) + float(delta)
    target = {name: target_val}

    if smooth:
        ok, msg = move_trap(target, DEFAULT_V_MAX, DEFAULT_A_MAX)
    else:
        ok, msg = move_linear(target)

    if ok:
        idx = NAME_TO_IDX[name]
        abs_deg = rel_to_abs(idx, target_val)
        return True, '{} jog 완료 | rel={:.1f}, abs={:.1f}'.format(
            name, target_val, abs_deg
        )
    return False, msg


def joint_neutral(name):
    if name not in NAME_TO_IDX:
        return False, '관절 이름 오류: {}'.format(name)

    scan_stop_internal(return_zero=False)

    ok, msg = move_trap({name: 0.0}, DEFAULT_V_MAX, DEFAULT_A_MAX)
    if ok:
        return True, '{} 중립 완료'.format(name)
    return False, msg


def recommend_minmax_message(name):
    """
    현재 위치를 기준으로 offset이 반영된 absolute angle을 계산하고,
    config.py에 넣을 min_deg 또는 max_deg 추천값을 출력.
    """
    if name not in NAME_TO_IDX:
        return '관절 이름 오류'

    idx = NAME_TO_IDX[name]
    rel = _current.get(name, 0.0)
    abs_deg = rel_to_abs(idx, rel)
    neutral_abs = rel_to_abs(idx, 0.0)

    if abs_deg >= neutral_abs:
        rec = abs_deg - SAFE_MARGIN_DEG
        return (
            '{} record | rel={:.1f}, abs={:.1f}\n'
            '중립 abs={:.1f}보다 큰 방향이므로 max_deg 후보입니다.\n'
            '추천: config.py에서 max_deg ≈ {:.1f}\n'
            '계산: {:.1f} - margin {:.1f}'
        ).format(name, rel, abs_deg, neutral_abs,
                 rec, abs_deg, SAFE_MARGIN_DEG)
    else:
        rec = abs_deg + SAFE_MARGIN_DEG
        return (
            '{} record | rel={:.1f}, abs={:.1f}\n'
            '중립 abs={:.1f}보다 작은 방향이므로 min_deg 후보입니다.\n'
            '추천: config.py에서 min_deg ≈ {:.1f}\n'
            '계산: {:.1f} + margin {:.1f}'
        ).format(name, rel, abs_deg, neutral_abs,
                 rec, abs_deg, SAFE_MARGIN_DEG)


def scan_start(name, direction):
    """
    브라우저에서 Start Up/Down을 누르면 스캔 상태만 켠다.
    실제 스캔 이동은 main loop의 scan_update()에서 수행한다.
    """
    if name not in NAME_TO_IDX:
        return False, '관절 이름 오류: {}'.format(name)
    if direction not in ('up', 'down'):
        return False, "direction은 'up' 또는 'down'"

    scan_stop_internal(return_zero=False)

    # 항상 0에서 시작
    ok, msg = move_trap({name: 0.0}, DEFAULT_V_MAX, DEFAULT_A_MAX)
    if not ok:
        return False, msg

    _scan['active'] = True
    _scan['name'] = name
    _scan['direction'] = direction
    _scan['last_ms'] = ticks_ms()
    _scan['last_msg'] = '{} scan {} 시작'.format(name, direction)

    return True, _scan['last_msg']


def scan_stop_internal(return_zero=True):
    """
    내부 stop. endpoint에서도 사용.
    """
    if not _scan.get('active'):
        return False, _scan.get('last_msg', 'scan 대기')

    name = _scan['name']
    _scan['active'] = False

    msg = recommend_minmax_message(name)
    _scan['last_msg'] = msg

    if return_zero:
        move_trap({name: 0.0}, DEFAULT_V_MAX, DEFAULT_A_MAX)
        _scan['last_msg'] = msg + '\n0도 중립으로 복귀 완료'

    return True, _scan['last_msg']


def scan_update():
    """
    서버 main loop에서 주기적으로 호출된다.
    HTTP 요청 없이 내부에서 부드럽게 한 스텝씩 이동한다.
    """
    if not _scan.get('active'):
        return

    now = ticks_ms()
    if ticks_diff(now, _scan['last_ms']) < SCAN_INTERVAL_MS:
        return

    _scan['last_ms'] = now

    name = _scan['name']
    direction = _scan['direction']

    if name not in NAME_TO_IDX:
        _scan['active'] = False
        _scan['last_msg'] = 'scan 오류: 관절 이름 오류'
        return

    cur = _current.get(name, 0.0)

    if direction == 'up':
        nxt = cur + SCAN_STEP_DEG
        limit_rel = 90.0
        if nxt > limit_rel:
            nxt = limit_rel
    else:
        nxt = cur - SCAN_STEP_DEG
        limit_rel = -90.0
        if nxt < limit_rel:
            nxt = limit_rel

    target = {name: nxt}
    ok, msg = check_pose(target)
    if not ok:
        _scan['active'] = False
        rec = recommend_minmax_message(name)
        _scan['last_msg'] = '스캔 안전 한계 도달\n{}\n{}'.format(msg, rec)
        move_trap({name: 0.0}, DEFAULT_V_MAX, DEFAULT_A_MAX)
        _scan['last_msg'] += '\n0도 중립으로 복귀 완료'
        return

    write_servo_rel(NAME_TO_IDX[name], nxt)
    _current[name] = nxt

    idx = NAME_TO_IDX[name]
    abs_deg = rel_to_abs(idx, nxt)
    _scan['last_msg'] = '{} scan {} | rel={:.1f}, abs={:.1f}'.format(
        name, direction, nxt, abs_deg
    )

    # ±90 끝까지 갔으면 기록 후 0으로 복귀
    if (direction == 'up' and nxt >= limit_rel) or \
       (direction == 'down' and nxt <= limit_rel):
        _scan['active'] = False
        rec = recommend_minmax_message(name)
        _scan['last_msg'] = rec + '\n±90도 끝 도달\n0도 중립으로 복귀 중...'
        move_trap({name: 0.0}, DEFAULT_V_MAX, DEFAULT_A_MAX)
        _scan['last_msg'] += '\n0도 중립으로 복귀 완료'


# ══════════════════════════════════════════════════════════
# 8. 03 / 04 Movement
# ══════════════════════════════════════════════════════════
def move_linear(target, step=1.0, delay_ms=25):
    """
    03_xyz_control.py 방식:
    현재 각도에서 목표 각도까지 일정 간격으로 보간 이동.
    속도/가속도 프로파일은 없음.
    """
    global _current

    ok, msg = check_pose(target)
    if not ok:
        return False, msg

    starts = {k: _current.get(k, 0.0) for k in target}
    diffs  = {k: target[k] - starts[k] for k in target}

    ticks = 1
    for d in diffs.values():
        n = abs(int(d / step))
        if n > ticks:
            ticks = n

    for i in range(ticks + 1):
        ratio = i / ticks
        frame = {k: starts[k] + diffs[k] * ratio for k in target}
        write_all_rel(frame)
        sleep_ms(delay_ms)

    _current.update(target)
    return True, 'Linear 이동 완료'


def move_trap(target, v_max=DEFAULT_V_MAX, a_max=DEFAULT_A_MAX):
    """
    04_smooth_control.py 방식:
    motion.py의 sync_profiles를 이용한 사다리꼴/삼각형 속도 프로파일 이동.
    """
    global _current

    ok, msg = check_pose(target)
    if not ok:
        return False, msg

    starts = {k: _current.get(k, 0.0) for k in target}

    try:
        profiles = sync_profiles(starts, target, float(v_max), float(a_max), DT)
    except Exception as e:
        return False, '프로파일 생성 오류: {}'.format(e)

    n = len(next(iter(profiles.values())))

    for i in range(n):
        t0 = ticks_ms()
        frame = {k: profiles[k][i] for k in profiles}
        write_all_rel(frame)

        wait = int(DT * 1000) - ticks_diff(ticks_ms(), t0)
        if wait > 0:
            sleep_ms(wait)

    _current.update(target)
    return True, 'Smooth 이동 완료'


def xyz_to_target(x, y, z):
    result = ik.ik(float(x), float(y), float(z))
    if result is None:
        return None, 'IK 도달 불가: ({},{},{})'.format(x, y, z)

    base, shoulder, elbow = result
    target = {
        'base'    : base,
        'shoulder': shoulder,
        'elbow'   : elbow,
        'wrist_r' : _current.get('wrist_r', 0.0),
        'wrist_p' : _current.get('wrist_p', 0.0),
    }

    ok, msg = check_pose(target)
    if not ok:
        return None, 'IK는 가능하지만 config 한계 초과: ' + msg

    return target, 'ok'


def live_set_xyz(x, y, z):
    target, msg = xyz_to_target(x, y, z)
    if target is None:
        return False, msg

    write_all_rel(target)
    _current.update(target)

    return True, 'Live XYZ | x={} y={} z={}'.format(x, y, z)


def move_to_xyz(x, y, z, mode='smooth', v_max=DEFAULT_V_MAX, a_max=DEFAULT_A_MAX):
    scan_stop_internal(return_zero=False)

    target, msg = xyz_to_target(x, y, z)
    if target is None:
        return False, msg

    if mode == 'linear':
        ok, msg = move_linear(target)
    else:
        ok, msg = move_trap(target, float(v_max), float(a_max))

    if not ok:
        return False, msg

    b = target['base']
    s = target['shoulder']
    e = target['elbow']
    fx, fy, fz = ik.fk(b, s, e)

    return True, '이동 완료 | FK=({},{},{}) | 각도 b={} s={} e={}'.format(
        fx, fy, fz, b, s, e
    )


def grip_move(rel_deg):
    scan_stop_internal(return_zero=False)
    return move_trap({'grip': float(rel_deg)}, DEFAULT_V_MAX, DEFAULT_A_MAX)


# ══════════════════════════════════════════════════════════
# 9. Status / HTML data
# ══════════════════════════════════════════════════════════
def get_status_text(target='all'):
    def line_for_servo(s):
        name = s['name']
        idx = NAME_TO_IDX[name]
        rel = _current.get(name, 0.0)
        abs_deg = rel_to_abs(idx, rel)
        rmin, rmax = allowed_rel_range(idx)
        return (
            '{} | rel={:.1f} | abs={:.1f} | offset={:+} | '
            'safe_abs={}~{} | safe_rel={:.1f}~{:.1f}'.format(
                name, rel, abs_deg, s['offset'],
                s['min_deg'], s['max_deg'], rmin, rmax
            )
        )

    lines = []

    if target == 'all':
        for s in config.SERVO:
            lines.append(line_for_servo(s))
    else:
        found = False
        for s in config.SERVO:
            if s['name'] == target:
                lines.append(line_for_servo(s))
                found = True
                break
        if not found:
            lines.append('알 수 없는 관절: {}'.format(target))

    lines.append('scan_active={} | {}'.format(
        _scan.get('active'), _scan.get('last_msg')
    ))

    return '\n'.join(lines)


def make_joint_info_json():
    arr = []
    for s in config.SERVO:
        name = s['name']
        idx = NAME_TO_IDX[name]
        rmin, rmax = allowed_rel_range(idx)

        arr.append({
            'name': name,
            'rel_min': round(rmin, 1),
            'rel_max': round(rmax, 1),
            'calib_min': s['min_deg'],
            'calib_max': s['max_deg'],
            'offset': s['offset'],
            'dir': s['dir'],
        })
    return json.dumps(arr)


# ══════════════════════════════════════════════════════════
# 10. HTML
# ══════════════════════════════════════════════════════════
HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MJYeom Robot Arm</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{--mint:#dff5f3;--mint2:#c7ebe8;--teal:#23c7c9;--tealDark:#11999e;--navy:#172033;--blue:#2563eb;--green:#18a66a;--orange:#ff9f1c;--red:#ff4d5e;--violet:#6d5dfc;--slate:#607080;--ink:#16181d;--sub:#606b72;--line:#d6e8e7;--paper:#fff;--soft:#f7fbfb}
body{font-family:Arial,'Noto Sans KR',sans-serif;background:linear-gradient(180deg,var(--mint),#eefaf9);color:var(--ink);padding:18px;min-height:100vh}
body:before{content:'C';position:fixed;left:18px;top:14px;width:36px;height:36px;border-radius:50%;background:#fff;color:var(--tealDark);font-weight:900;font-size:25px;line-height:36px;text-align:center;box-shadow:0 4px 12px rgba(17,153,158,.16);z-index:0}
.pageTitle{max-width:620px;margin:10px auto 16px;background:var(--paper);border:10px solid var(--mint2);border-radius:10px;padding:24px 20px;text-align:center;box-shadow:0 10px 28px rgba(20,90,92,.10);position:relative;overflow:hidden}.pageTitle:after{content:'';position:absolute;left:0;right:0;bottom:0;height:6px;background:linear-gradient(90deg,var(--tealDark),var(--red),var(--orange))}
h1{font-size:1.72rem;line-height:1.1;letter-spacing:-.05em;color:var(--ink);font-weight:900;margin-bottom:7px}.subtitle{font-size:.82rem;color:var(--sub);font-weight:800}.subtitle b{color:var(--tealDark)}
.card{background:var(--paper);border:1px solid var(--line);border-radius:10px;padding:16px;margin:0 auto 14px;max-width:620px;box-shadow:0 6px 20px rgba(20,80,80,.07)}
.card h2{font-size:1rem;letter-spacing:-.03em;color:var(--ink);font-weight:900;border-left:8px solid var(--red);background:#fff;padding:5px 0 6px 10px;margin-bottom:13px;border-radius:0}.card h2:after{content:'';display:block;width:44px;height:3px;background:var(--teal);margin-top:7px;border-radius:99px}
label{display:flex;justify-content:space-between;font-size:.84rem;color:#30363a;margin:10px 0 4px;font-weight:900}label span{color:var(--red);font-weight:900}
input[type=range]{width:100%;margin-bottom:9px;accent-color:var(--tealDark)}select,input{width:100%;background:#fff;color:var(--ink);border:1.5px solid #b9d2d2;border-radius:8px;padding:9px;font-weight:800}select:focus,input:focus{outline:2px solid var(--teal);border-color:var(--tealDark)}
.row{display:flex;gap:8px;margin-top:8px}button{flex:1;padding:11px 8px;border:0;border-radius:9px;font-weight:900;color:#fff;cursor:pointer;box-shadow:0 4px 0 rgba(0,0,0,.16);letter-spacing:-.02em}button:active{transform:translateY(1px);box-shadow:0 2px 0 rgba(0,0,0,.18)}
.b1{background:linear-gradient(135deg,var(--tealDark),var(--blue))}.b2{background:linear-gradient(135deg,var(--green),#35d58a)}.b3{background:linear-gradient(135deg,var(--orange),#ff6b35)}.b4{background:linear-gradient(135deg,var(--red),#d9283f)}.b5{background:linear-gradient(135deg,var(--navy),var(--violet))}.b6{background:linear-gradient(135deg,var(--slate),#93a1ad)}
#st,#statbox{max-width:620px;margin:0 auto 12px;background:var(--navy);color:#f2ffff;border:3px solid var(--teal);border-radius:8px;padding:12px;min-height:44px;font-size:.84rem;white-space:pre-wrap;font-family:monospace}#statbox{display:none;color:#e5e7eb;font-size:.76rem}
.localbox{background:#f1f8f8;color:#1d292d;border:1.5px solid #c8dddd;border-left:6px solid var(--teal);border-radius:8px;padding:11px;min-height:40px;font-family:monospace;font-size:.77rem;white-space:pre-wrap;margin-top:9px}.localbox.empty{color:#879397;background:#f8fbfb;border-left-color:#b8cccc}
.small{font-size:.76rem;color:#5b666b;line-height:1.45;margin-top:9px}.jointBox{border:1.5px solid #d5e7e7;border-radius:9px;padding:11px;margin-bottom:10px;background:var(--soft)}.result{font-size:.79rem;color:#c01021;font-weight:900;margin-top:5px;white-space:pre-wrap}.info{font-size:.75rem;color:#69757b;line-height:1.4}.checkrow{display:flex;align-items:center;gap:8px;margin:9px 0;color:#30363a;font-size:.85rem;font-weight:900}.checkrow input{width:auto;accent-color:var(--red)}
@media(max-width:520px){body{padding:12px}.pageTitle{padding:18px 14px;border-width:8px}.row{gap:6px}button{font-size:.8rem;padding:10px 5px}h1{font-size:1.42rem}.card{padding:14px}}
</style>
</head>
<body>
<div class="pageTitle"><h1>MJYeom Robot Arm</h1><div class="subtitle"><b>AP Web Control</b> · RobotArm Wi‑Fi · http://192.168.4.1</div></div>

<div class="card">
<h2>00 Neutral / Home</h2>
<div class="row">
<button class="b3" onclick="post('/neutral_raw',{})">Raw Neutral</button>
<button class="b2" onclick="post('/home_calibrated',{})">Calibrated Home</button>
</div>
<label>Status Target</label>
<select id="statusTarget"></select>
<div class="row">
<button class="b6" onclick="showStatus()">Status</button>
<button class="b4" onclick="clearLogs()">Clear Log</button>
</div>
<div id="statusLocal" class="localbox empty">Status output will appear here.</div>
<p class="small">
Raw Neutral은 원본 00_neutral.py처럼 offset을 고려하지 않습니다.<br>
Calibrated Home은 config.py의 offset을 반영합니다.
</p>
</div>

<div class="card">
<h2>01 Offset Calibration</h2>
<div class="row">
<button class="b6" onclick="resetAllCalibCenters()">All Center</button>
</div>
<div id="calibSliders"></div>
<p class="small">
일반 관절은 기준 자세가 맞는 software_abs를 찾아 recommended offset을 확인합니다.<br>
grip은 offset보다 열림/닫힘 방향이 중요하므로 rel 기준으로 표시합니다.<br>
일반 관절 공식: offset = software_abs - 90
</p>
</div>

<div class="card">
<h2>02 Angle Test - Jog / Scan</h2>
<label>Joint</label>
<select id="joint"></select>
<div class="row">
<button class="b4" onclick="jog(-10)">-10°</button>
<button class="b4" onclick="jog(-5)">-5°</button>
<button class="b2" onclick="joint0()">0°</button>
<button class="b1" onclick="jog(5)">+5°</button>
<button class="b1" onclick="jog(10)">+10°</button>
</div>
<div class="row">
<button class="b5" onclick="startScan('up')">Start Up</button>
<button class="b5" onclick="startScan('down')">Start Down</button>
<button class="b4" onclick="stopScan()">Stop / Record</button>
</div>
<div id="scanLocal" class="localbox empty">Angle test output will appear here.</div>
<p class="small">
Start Up/Down은 서버 내부에서 연속적으로 움직여 버튼 반복 방식보다 부드럽습니다.<br>
위험해 보이는 지점에서 Stop을 누르면 offset이 반영된 abs 각도 기준으로 min/max 추천값이 출력되고 0으로 복귀합니다.
</p>
</div>

<div class="card">
<h2>03 / 04 XYZ Control</h2>
<label>X cm <span id="xv">18</span></label>
<input type="range" id="x" min="5" max="28" value="18" step="1" oninput="xyzChanged()">
<label>Y cm <span id="yv">0</span></label>
<input type="range" id="y" min="-20" max="20" value="0" step="1" oninput="xyzChanged()">
<label>Z cm <span id="zv">18</span></label>
<input type="range" id="z" min="8" max="30" value="18" step="1" oninput="xyzChanged()">

<div class="checkrow">
<input type="checkbox" id="liveXYZ">
<span>Live XYZ 사용: 슬라이더를 움직이면 즉시 반영</span>
</div>

<label>v_max deg/s <span id="vv">60</span></label>
<input type="range" id="vmax" min="20" max="160" value="60" step="10"
 oninput="document.getElementById('vv').textContent=this.value">
<label>a_max deg/s² <span id="av">120</span></label>
<input type="range" id="amax" min="40" max="300" value="120" step="20"
 oninput="document.getElementById('av').textContent=this.value">

<div class="row">
<button class="b5" onclick="moveXYZ('linear')">03 Linear</button>
<button class="b1" onclick="moveXYZ('smooth')">04 Smooth</button>
</div>
<p class="small">
03 Linear: 목표 각도까지 일정 간격으로 나누어 이동합니다.<br>
04 Smooth: v_max, a_max를 이용해 천천히 출발하고 천천히 멈춥니다.
</p>
</div>

<div class="card">
<h2>Gripper</h2>
<div class="row">
<button class="b2" onclick="grip(60)">Open</button>
<button class="b4" onclick="grip(0)">Close</button>
<button class="b6" onclick="grip(0)">Neutral</button>
</div>
<p class="small">
이 설정은 config.py에서 grip의 dir=-1일 때 기준입니다.<br>
rel=0 → abs=90 닫힘, rel=+30 → abs=30 열림
</p>
</div>

<div id="st">Ready</div>
<div id="statbox"></div>

<script>
const JOINTS = __JOINTS__;
const liveTimers = {};
const liveLast = {};
let scanPoll = null;
let xyzTimer = null;
let xyzLast = 0;

function setSt(t){document.getElementById('st').textContent=t;}

async function post(path,data,silent=false){
  if(!silent) setSt('sending...');
  try{
    let r=await fetch(path,{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify(data)
    });
    let j=await r.json();
    if(!silent || !j.ok) setSt((j.ok?'OK: ':'ERR: ')+j.msg);
    return j;
  }catch(e){
    setSt('error: '+e);
    return {ok:false,msg:String(e)};
  }
}

function makeUI(){
  const calib = document.getElementById('calibSliders');
  const sel = document.getElementById('joint');
  const statusSel = document.getElementById('statusTarget');

  const allOpt = document.createElement('option');
  allOpt.value = 'all';
  allOpt.textContent = '전체 관절정보';
  statusSel.appendChild(allOpt);

  JOINTS.forEach(j=>{
    const opt = document.createElement('option');
    opt.value = j.name;
    opt.textContent = j.name;
    sel.appendChild(opt);

    const sopt = document.createElement('option');
    sopt.value = j.name;
    sopt.textContent = j.name + ' 정보';
    statusSel.appendChild(sopt);

    const box = document.createElement('div');
    box.className = 'jointBox';

    if(j.name === 'grip'){
      const startRel = 0;
      box.innerHTML = `
        <label>grip rel(deg) - 열림 제어
          <span id="grip_rel_v">${startRel}</span>
        </label>
        <input type="range" id="grip_rel_s"
          min="${j.rel_min}" max="${j.rel_max}" value="${startRel}" step="1"
          oninput="calibGripRel(this.value)">
        <div class="info">
          safe_rel: ${j.rel_min}° ~ ${j.rel_max}° |
          current config offset: ${j.offset} |
          dir: ${j.dir}
        </div>
        <div class="row">
          <button class="b6" onclick="resetCalibCenter('grip')">Center</button>
        </div>
        <div class="result" id="grip_offset_r">
          rel=0 → 닫힘 기준 / rel 증가 → 열림 방향
        </div>
      `;
    }else{
      box.innerHTML = `
        <label>${j.name} software abs(deg)
          <span id="${j.name}_abs_v">90</span>
        </label>
        <input type="range" id="${j.name}_abs_s"
          min="${j.calib_min}" max="${j.calib_max}" value="90" step="1"
          oninput="calibJoint('${j.name}', this.value)">
        <div class="info">
          safe_rel: ${j.rel_min}° ~ ${j.rel_max}° |
          current config offset: ${j.offset} |
          dir: ${j.dir}
        </div>
        <div class="row">
          <button class="b6" onclick="resetCalibCenter('${j.name}')">Center</button>
        </div>
        <div class="result" id="${j.name}_offset_r">
          recommended offset = 0
        </div>
      `;
    }

    calib.appendChild(box);
  });
}

function calibJoint(name,value){
  document.getElementById(name+'_abs_v').textContent = value;
  const offset = (+value - 90);
  document.getElementById(name+'_offset_r').textContent =
    'software_abs='+value+'° | error='+
    (offset>=0?'+':'')+offset.toFixed(1)+
    '° | recommended offset='+(offset>=0?'+':'')+offset.toFixed(1);

  const now = Date.now();
  const gap = 100;

  function send(){
    liveLast[name] = Date.now();
    liveTimers[name] = null;
    post('/calib_joint',{name:name,abs:+value},true);
  }

  if(!liveLast[name] || now - liveLast[name] > gap){
    send();
  }else{
    if(liveTimers[name]) clearTimeout(liveTimers[name]);
    liveTimers[name] = setTimeout(send, gap - (now - liveLast[name]));
  }
}

function calibGripRel(value){
  document.getElementById('grip_rel_v').textContent = value;

  const rel = +value;
  document.getElementById('grip_offset_r').textContent =
    'grip rel='+rel.toFixed(1)+'° | rel=0 닫힘 / rel 증가 열림';

  const name = 'grip';
  const now = Date.now();
  const gap = 100;

  function send(){
    liveLast[name] = Date.now();
    liveTimers[name] = null;
    post('/calib_grip_rel',{rel:rel},true);
  }

  if(!liveLast[name] || now - liveLast[name] > gap){
    send();
  }else{
    if(liveTimers[name]) clearTimeout(liveTimers[name]);
    liveTimers[name] = setTimeout(send, gap - (now - liveLast[name]));
  }
}

function sliderCenterValue(el){
  const min = parseFloat(el.min);
  const max = parseFloat(el.max);
  const step = parseFloat(el.step || '1');
  let mid = (min + max) / 2;

  if(step > 0){
    mid = Math.round(mid / step) * step;
  }

  if(mid < min) mid = min;
  if(mid > max) mid = max;

  return mid;
}

function resetCalibCenter(name){
  if(name === 'grip'){
    const el = document.getElementById('grip_rel_s');
    const mid = sliderCenterValue(el);
    el.value = mid;
    calibGripRel(mid);
  }else{
    const el = document.getElementById(name + '_abs_s');
    const mid = sliderCenterValue(el);
    el.value = mid;
    calibJoint(name, mid);
  }
}

function resetAllCalibCenters(){
  JOINTS.forEach(j => resetCalibCenter(j.name));
  setSt('Offset Calibration sliders centered');
}

function jog(d){
  stopScanPoll();
  post('/joint',{name:document.getElementById('joint').value,delta:d});
}

function joint0(){
  stopScanPoll();
  post('/joint0',{name:document.getElementById('joint').value});
}

function stopScanPoll(){
  if(scanPoll){
    clearInterval(scanPoll);
    scanPoll = null;
  }
}

async function startScan(direction){
  stopScanPoll();
  const name = document.getElementById('joint').value;
  const j = await post('/scan_start',{name:name,direction:direction});
  const scanBox = document.getElementById('scanLocal');
  scanBox.classList.remove('empty');
  scanBox.textContent = (j.ok?'OK: ':'ERR: ')+j.msg;
  if(!j.ok) return;

  scanPoll = setInterval(async ()=>{
    const s = await post('/scan_status',{},true);
    const msg = (s.ok?'OK: ':'ERR: ')+s.msg;
    document.getElementById('scanLocal').classList.remove('empty');
    document.getElementById('scanLocal').textContent = msg;
    setSt('Scan running...');
    if(s.msg.indexOf('복귀 완료') >= 0 || s.msg.indexOf('대기') >= 0){
      stopScanPoll();
    }
  }, 300);
}

async function stopScan(){
  stopScanPoll();
  const s = await post('/scan_stop',{},true);
  const scanBox = document.getElementById('scanLocal');
  scanBox.classList.remove('empty');
  scanBox.textContent = (s.ok?'OK: ':'ERR: ')+s.msg;
  setSt('Scan stopped / recorded');
}

async function showStatus(){
  const target = document.getElementById('statusTarget').value;
  const j = await post('/status',{target:target},true);
  const box = document.getElementById('statusLocal');
  box.classList.remove('empty');
  box.textContent = j.msg || 'status 없음';
  setSt((j.ok?'OK: ':'ERR: ')+(j.msg ? 'Status updated' : 'status 없음'));
}

function clearLogs(){
  const statusBox = document.getElementById('statusLocal');
  const scanBox = document.getElementById('scanLocal');
  statusBox.textContent = 'Status output will appear here.';
  statusBox.classList.add('empty');
  scanBox.textContent = 'Angle test output will appear here.';
  scanBox.classList.add('empty');
  document.getElementById('st').textContent = 'Ready';
  const bottom = document.getElementById('statbox');
  if(bottom){
    bottom.textContent = '';
    bottom.style.display = 'none';
  }
}

function xyzChanged(){
  const x = document.getElementById('x').value;
  const y = document.getElementById('y').value;
  const z = document.getElementById('z').value;

  document.getElementById('xv').textContent=x;
  document.getElementById('yv').textContent=y;
  document.getElementById('zv').textContent=z;

  if(!document.getElementById('liveXYZ').checked) return;

  const now = Date.now();
  const gap = 150;

  function sendXYZ(){
    xyzLast = Date.now();
    xyzTimer = null;
    post('/live_xyz',{x:+x,y:+y,z:+z},true);
  }

  if(!xyzLast || now - xyzLast > gap){
    sendXYZ();
  }else{
    if(xyzTimer) clearTimeout(xyzTimer);
    xyzTimer = setTimeout(sendXYZ, gap - (now - xyzLast));
  }
}

function moveXYZ(mode){
  stopScanPoll();
  post('/move',{
    mode:mode,
    x:+document.getElementById('x').value,
    y:+document.getElementById('y').value,
    z:+document.getElementById('z').value,
    v_max:+document.getElementById('vmax').value,
    a_max:+document.getElementById('amax').value
  });
}

function grip(v){
  stopScanPoll();
  post('/grip',{v:v});
}

makeUI();
</script>
</body>
</html>"""

HTML = HTML_TEMPLATE.replace('__JOINTS__', make_joint_info_json())


# ══════════════════════════════════════════════════════════
# 11. HTTP Server
# ══════════════════════════════════════════════════════════
def parse_request(raw):
    try:
        text = raw.decode('utf-8')
        first = text.split('\r\n')[0]
        method, path = first.split(' ')[:2]

        body = ''
        if '\r\n\r\n' in text:
            body = text.split('\r\n\r\n', 1)[1]

        data = json.loads(body) if body.strip() else {}
        return method, path, data
    except Exception as e:
        print('요청 파싱 오류:', e)
        return 'GET', '/', {}


def send_response(conn, status, ctype, body):
    if isinstance(body, str):
        body = body.encode('utf-8')
    header = (
        'HTTP/1.1 {}\r\n'
        'Content-Type:{};charset=utf-8\r\n'
        'Content-Length:{}\r\n'
        'Connection:close\r\n\r\n'
    ).format(status, ctype, len(body))
    conn.send(header.encode())
    conn.send(body)


def send_json(conn, ok, msg):
    send_response(conn, '200 OK', 'application/json',
                  json.dumps({'ok': ok, 'msg': msg}))


def handle(method, path, data, conn):
    if method == 'GET' and path == '/':
        send_response(conn, '200 OK', 'text/html', HTML)

    elif method == 'POST' and path == '/neutral_raw':
        ok, msg = raw_neutral()
        send_json(conn, ok, msg)

    elif method == 'POST' and path == '/home_calibrated':
        ok, msg = calibrated_home(True)
        send_json(conn, ok, msg)

    elif method == 'POST' and path == '/status':
        target = data.get('target', 'all')
        send_json(conn, True, get_status_text(target))

    elif method == 'POST' and path == '/calib_joint':
        name = data.get('name', 'base')
        abs_deg = float(data.get('abs', 90))
        ok, msg = calib_set_joint(name, abs_deg)
        send_json(conn, ok, msg)

    elif method == 'POST' and path == '/calib_grip_rel':
        rel_deg = float(data.get('rel', 0))
        ok, msg = calib_set_grip_rel(rel_deg)
        send_json(conn, ok, msg)

    elif method == 'POST' and path == '/joint':
        name  = data.get('name', 'base')
        delta = float(data.get('delta', 0))
        ok, msg = jog_joint(name, delta, True)
        send_json(conn, ok, msg)

    elif method == 'POST' and path == '/joint0':
        name = data.get('name', 'base')
        ok, msg = joint_neutral(name)
        send_json(conn, ok, msg)

    elif method == 'POST' and path == '/scan_start':
        name = data.get('name', 'base')
        direction = data.get('direction', 'up')
        ok, msg = scan_start(name, direction)
        send_json(conn, ok, msg)

    elif method == 'POST' and path == '/scan_stop':
        ok, msg = scan_stop_internal(return_zero=True)
        send_json(conn, True, msg)

    elif method == 'POST' and path == '/scan_status':
        send_json(conn, True, _scan.get('last_msg', 'scan 대기'))

    elif method == 'POST' and path == '/move':
        x = float(data.get('x', 18))
        y = float(data.get('y', 0))
        z = float(data.get('z', 18))
        mode  = data.get('mode', 'smooth')
        v_max = float(data.get('v_max', DEFAULT_V_MAX))
        a_max = float(data.get('a_max', DEFAULT_A_MAX))

        ok, msg = move_to_xyz(x, y, z, mode, v_max, a_max)
        send_json(conn, ok, msg)

    elif method == 'POST' and path == '/live_xyz':
        x = float(data.get('x', 18))
        y = float(data.get('y', 0))
        z = float(data.get('z', 18))
        ok, msg = live_set_xyz(x, y, z)
        send_json(conn, ok, msg)

    elif method == 'POST' and path == '/grip':
        v = float(data.get('v', 0))
        ok, msg = grip_move(v)
        if ok:
            msg = '그리퍼 이동 완료: rel={}deg'.format(v)
        send_json(conn, ok, msg)

    else:
        send_response(conn, '404 Not Found', 'text/plain', 'Not Found')


# ══════════════════════════════════════════════════════════
# 12. Start
# ══════════════════════════════════════════════════════════
if HW_OK:
    raw_neutral()
    sleep_ms(500)

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('0.0.0.0', 80))
srv.listen(5)

# scan_update를 위해 accept timeout 설정
try:
    srv.settimeout(0.05)
except Exception:
    pass

print('\n웹서버 시작')
print('휴대폰/PC에서 WiFi RobotArm 연결 후 http://192.168.4.1 접속')
print('여러 대 접속은 가능하지만 실제 조작은 1대만 권장\n')

while True:
    conn = None
    try:
        scan_update()

        try:
            conn, addr = srv.accept()
        except OSError:
            continue

        raw = conn.recv(4096)
        if raw:
            method, path, data = parse_request(raw)
            print(method, path, data)
            handle(method, path, data, conn)

    except Exception as e:
        print('서버 오류:', e)
    finally:
        if conn:
            conn.close()


