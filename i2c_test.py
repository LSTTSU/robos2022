from smbus import SMBus
import time

stm_main = 0x29
i2cbus = SMBus(1)

while True:
    i2cbus.write_byte(stm_main,0x23)
    time.sleep(0.02)
    i2cbus.write_byte(stm_main,0x24)
    time.sleep(0.02)
    i2cbus.write_byte(stm_main,0x25)
    time.sleep(0.02)
    i2cbus.write_byte(stm_main,0x26)

    time.sleep(0.02)
