# 00_neutral.py
from machine import I2C, Pin
from pca9685 import PCA9685
import config
                  
i2c = I2C(0, scl=Pin(config.I2C_SCL),
              sda=Pin(config.I2C_SDA), freq=400_000)
pca = PCA9685(i2c, config.PCA_ADDR)

# 서보별 중립 펄스폭 (us)
# 조립 전 눈으로 확인하면서 맞추세요
NEUTRAL_US = {
    0: 1500,   # 베이스    HJ S3315
    1: 1500,   # 어깨      MG996R
    2: 1500,   # 팔꿈치    MG996R
    3: 1450,   # 손목회전  MG90S
    4: 1450,   # 손목상하  MG90S
    5: 1450,   # 그리퍼    MG90S
}
 
for ch, us in NEUTRAL_US.items():
    pca.set_us(config.SERVO_CH[ch], us)
    print(f'CH{ch} ({config.SERVO[ch]["name"]}) → {us}us 완료')

print('\n전체 완료 - 이 상태에서 조립하세요')
