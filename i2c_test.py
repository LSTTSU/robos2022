from smbus import SMBus
import time

stm_main = 0x29
i2cbus = SMBus(1)
stm_sleep_time = 0.02

while True:
    i2cbus.write_byte(stm_main,0x00)
    time.sleep(stm_sleep_time)
    i2cbus.write_byte(stm_main,0x0f)
    time.sleep(stm_sleep_time)
    i2cbus.write_byte(stm_main,0x2c)
    time.sleep(stm_sleep_time)
    i2cbus.write_byte(stm_main,0xff)

    time.sleep(0.02)
