# motion.py
# 속도 프로파일 계산 담당
# ESP32 에서 실행

def trapezoidal(start, end, v_max, a_max, dt):
    """
    사다리꼴 속도 프로파일로 이동 경로 생성

    start  : 시작 각도 (도)
    end    : 목표 각도 (도)
    v_max  : 최대 속도 (도/초)
    a_max  : 최대 가속도 (도/초²)
    dt     : 샘플 간격 (초)

    반환: [각도, 각도, ...] 리스트
    """
    dist = abs(end - start)              
    sign = 1 if end > start else -1

    if dist < 0.1:
        return [end]

    # 가속 구간 시간 (즉, 최대 속도까지 도달하는 시간)
    t_acc = v_max / a_max

    # 가속 구간 이동 거리
    d_acc = 0.5 * a_max * t_acc * t_acc

    if 2 * d_acc > dist:
        # 거리가 짧아서 최대 속도 못 도달 → 삼각형 프로파일
        t_acc  = (dist / a_max) ** 0.5
        v_peak = a_max * t_acc
        t_flat = 0.0
    else:
        # 사다리꼴 프로파일
        v_peak = v_max
        d_flat = dist - 2 * d_acc
        t_flat = d_flat / v_max

    t_total = 2 * t_acc + t_flat

    # 시간별 위치 샘플링
    points = []
    t = 0.0
    while t <= t_total + dt:
        if t <= t_acc:
            # 가속 구간
            s = 0.5 * a_max * t * t
        elif t <= t_acc + t_flat:
            # 등속 구간
            s = d_acc + v_peak * (t - t_acc)
        else:
            # 감속 구간
            td = t - t_acc - t_flat
            s  = d_acc + v_peak * t_flat + v_peak * td - 0.5 * a_max * td * td

        s = max(0.0, min(s, dist))
        points.append(start + sign * s)
        t += dt

    # 마지막은 정확히 end 로
    if not points or abs(points[-1] - end) > 0.1:
        points.append(end)

    return points


def sync_profiles(starts, ends, v_max, a_max, dt):
    """
    여러 축을 동시에 시작해서 동시에 끝나는 프로파일 생성

    가장 오래 걸리는 축에 맞춰서 나머지 축 속도를 조절

    starts : {'base': 0, 'shoulder': 10, ...}
    ends   : {'base': 30, 'shoulder': 45, ...}
    반환   : {'base': [점들], 'shoulder': [점들], ...}
    """

    # 각 축 개별 포인트 생성
    profiles = {}
    max_len  = 0

    for key in starts:
        pts = trapezoidal(
            starts[key], ends[key], v_max, a_max, dt
        )
        profiles[key] = pts
        max_len = max(max_len, len(pts))

    # 모든 축을 같은 길이로 맞춤
    # (짧은 축은 마지막 값으로 패딩)
    for key in profiles:
        while len(profiles[key]) < max_len:
            profiles[key].append(profiles[key][-1])

    return profiles
