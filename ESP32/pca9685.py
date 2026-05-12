# pca9685.py
from time import sleep_ms

class PCA9685:
    def __init__(self, i2c, addr=0x40, freq=50):
        self.i2c  = i2c
        self.addr = addr

        found = i2c.scan()
        print("I2C scan:", [hex(x) for x in found])
        if addr not in found:
            raise OSError(f"PCA9685 not found. addr=0x{addr:02X}")

        self._write(0x00, 0x10)                              # sleep
        pre = round(25_000_000 / (4096 * freq)) - 1
        self._write(0xFE, pre)                               # prescale → 50Hz
        self._write(0x00, 0x00)                              # wake
        sleep_ms(5)
        self._write(0x00, 0xA1)                              # auto-increment ON
        print(f"PCA9685 ready  freq={freq}Hz  prescale={pre}")

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val & 0xFF]))

    def set_us(self, ch, us):
        """채널 ch 에 펄스폭 us 마이크로초 출력"""
        off = int(us * 4096 * 50 / 1_000_000)
        off = max(0, min(4095, off))
        reg = 0x06 + ch * 4
        self.i2c.writeto_mem(self.addr, reg,
            bytes([0, 0, off & 0xFF, off >> 8]))

    def all_off(self):
        """전체 채널 출력 중단"""
        for ch in range(16):
            self.i2c.writeto_mem(self.addr, 0x06 + ch * 4,
                bytes([0, 0, 0, 0]))
