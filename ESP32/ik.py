# ik.py
# 좌표 ↔ 각도 변환 계산만 담당
# ESP32에서 실행 (math 모듈 사용)

import math
import config

L = config.LINK


# ── FK (순기구학) ────────────────────────────────────────
# 각도 → 좌표
# "지금 서보 각도로 그리퍼가 어디 있지?" 계산

def fk(base_deg, shoulder_deg, elbow_deg):
    """
    서보 각도 3개 → 그리퍼 끝 위치 (x, y, z) cm

    각도 기준:
      base     : 0 = 정면 / 양수 = 왼쪽 / 음수 = 오른쪽
      shoulder : 0 = 수평 / 양수 = 위
      elbow    : 0 = 일자 / 음수 = 위로 접힘
    """
    b = math.radians(base_deg)
    s = math.radians(shoulder_deg)
    e = math.radians(elbow_deg)

    # 측면 평면에서 수평 거리, 수직 높이 계산
    r = (L['a2'] * math.cos(s)
       + L['a3'] * math.cos(s + e)
       + L['a5'])
    z = (L['d1']
       + L['a2'] * math.sin(s)
       + L['a3'] * math.sin(s + e))

    # 베이스 회전 적용
    x = r * math.cos(b)
    y = r * math.sin(b)

    return round(x, 2), round(y, 2), round(z, 2)


# ── IK (역기구학) ────────────────────────────────────────
# 좌표 → 각도
# "그리퍼를 저기 보내려면 서보를 얼마나 돌려야 해?" 계산

def ik(x, y, z, elbow_up=True):
    """
    목표 위치 (x, y, z) cm → 서보 각도 3개

    elbow_up=True  : 팔꿈치가 위로 올라가는 자세 (기본)
    elbow_up=False : 팔꿈치가 아래로 내려가는 자세

    반환값:
      (base_deg, shoulder_deg, elbow_deg)  성공
      None                                  도달 불가
    """

    # ── 1단계: 베이스 각도 ─────────────────────────────
    # x, y 평면에서 목표 방향각
    base = math.degrees(math.atan2(y, x))

    # ── 2단계: 손목까지의 수평 거리 & 높이 ────────────
    # 그리퍼 끝 좌표에서 손목 길이(a5)만큼 빼서
    # 어깨~손목 문제로 단순화
    r = math.sqrt(x*x + y*y) - L['a5']
    h = z - L['d1']

    # ── 3단계: 도달 가능 여부 확인 ────────────────────
    dist   = math.sqrt(r*r + h*h)
    max_r  = L['a2'] + L['a3']
    min_r  = abs(L['a2'] - L['a3'])

    if dist > max_r:
        print(f'  너무 멀음: dist={dist:.1f} max={max_r:.1f}')
        return None
    if dist < min_r:
        print(f'  너무 가까움: dist={dist:.1f} min={min_r:.1f}')
        return None
    if r < 0:
        print(f'  그리퍼 뒤쪽은 불가')
        return None

    # ── 4단계: 팔꿈치 각도 (코사인 법칙) ──────────────
    cos_e = ((dist*dist - L['a2']**2 - L['a3']**2)
             / (2 * L['a2'] * L['a3']))
    cos_e = max(-1.0, min(1.0, cos_e))   # 수치 오차 방지

    elbow = math.degrees(math.acos(cos_e))
    if elbow_up:
        elbow = -elbow    # 음수 = 팔꿈치 위로

    # ── 5단계: 어깨 각도 ──────────────────────────────
    alpha = math.atan2(h, r)
    cos_b = ((L['a2']**2 + dist**2 - L['a3']**2)
             / (2 * L['a2'] * dist))
    cos_b = max(-1.0, min(1.0, cos_b))
    beta  = math.acos(cos_b)

    if elbow_up:
        shoulder = math.degrees(alpha + beta)
    else:
        shoulder = math.degrees(alpha - beta)

    # ── 6단계: FK로 검증 ───────────────────────────────
    fx, fy, fz = fk(base, shoulder, elbow)
    err = math.sqrt((fx-x)**2 + (fy-y)**2 + (fz-z)**2)
    if err > 1.5:
        print(f'  IK 오차 큼: {err:.2f}cm')
        return None

    return (round(base,    1),
            round(shoulder,1),
            round(elbow,   1))


# ── 작업 공간 확인 ───────────────────────────────────────

def is_reachable(x, y, z):
    """해당 좌표가 팔이 닿는지 빠르게 확인 (IK 계산 없이)"""
    r    = math.sqrt(x*x + y*y) - L['a5']
    h    = z - L['d1']
    dist = math.sqrt(r*r + h*h)
    return (abs(L['a2'] - L['a3']) < dist < L['a2'] + L['a3']
            and r >= 0)
