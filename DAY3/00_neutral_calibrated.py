# 00_neutral_calibrated.py
# config.py의 offset을 적용해서 보정된 중립 위치로 이동
# Thonny에서 실행

from machine import I2C, Pin
from utime import sleep_ms
from pca9685 import PCA9685
import config

# ── I2C / PCA9685 초기화 ─────────────────────
i2c = I2C(0, scl=Pin(config.I2C_SCL),
              sda=Pin(config.I2C_SDA), freq=400_000)
pca = PCA9685(i2c, config.PCA_ADDR)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def deg_to_us(idx, abs_deg):
    cfg = config.SERVO[idx]
    return int(cfg['min_us'] +
               (cfg['max_us'] - cfg['min_us']) * abs_deg / 180.0)


print("=" * 55)
print("보정된 중립 위치로 이동")
print("=" * 55)

for idx, cfg in enumerate(config.SERVO):
    # 중립 입력 90도 기준
    # abs_deg = 90 + offset
    abs_deg = 90 + cfg['offset']

    # 안전 범위 제한
    abs_deg = clamp(abs_deg, cfg['min_deg'], cfg['max_deg'])

    us = deg_to_us(idx, abs_deg)

    pca.set_us(config.SERVO_CH[idx], us)

    print(f"CH{idx} ({cfg['name']:10s}) "
          f"offset={cfg['offset']:+4d}  "
          f"neutral_abs={abs_deg:6.1f}deg  "
          f"us={us:4d}")

    sleep_ms(300)

print("=" * 55)
print("전체 완료 - offset이 반영된 중립 위치입니다.")
print("=" * 55)
