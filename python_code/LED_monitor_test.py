import spidev                       #SPI通信用のモジュールをインポート
import time                         #時間制御用のモジュールをインポート
import sys                          #sysモジュールをインポート

spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 1000000

spi.xfer2([0x09,0x00])
spi.xfer2([0x0a,0x04])
spi.xfer2([0x0b,0x07])
spi.xfer2([0x0c,0x01])
spi.xfer2([0x0f,0x00])
spi.xfer2([0x00,0x00])
spi.xfer2([0x00,0x00])
spi.xfer2([0x00,0x00])

time.sleep(0.3);

while True:

    time.sleep(1.0)

    spi.xfer3([0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00])
    spi.xfer3([0x02,0x00,0x02,0x01,0x02,0x80,0x02,0x00])
    spi.xfer3([0x03,0x00,0x03,0x02,0x03,0x40,0x03,0x00])
    spi.xfer3([0x04,0x00,0x04,0x00,0x04,0x00,0x04,0x00])
    spi.xfer3([0x05,0x00,0x05,0x00,0x05,0x00,0x05,0x00])
    spi.xfer3([0x06,0x00,0x06,0x0a,0x06,0x50,0x06,0x00])
    spi.xfer3([0x07,0x00,0x07,0x04,0x07,0x20,0x07,0x00])
    spi.xfer3([0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00])

    time.sleep(1.0)

    spi.xfer3([0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00])
    spi.xfer3([0x02,0x00,0x02,0x03,0x02,0xc0,0x02,0x00])
    spi.xfer3([0x03,0x00,0x03,0x02,0x03,0x40,0x03,0x00])
    spi.xfer3([0x04,0x00,0x04,0x23,0x04,0xc4,0x04,0x00])
    spi.xfer3([0x05,0x00,0x05,0x10,0x05,0x08,0x05,0x00])
    spi.xfer3([0x06,0x00,0x06,0x08,0x06,0x10,0x06,0x00])
    spi.xfer3([0x07,0x00,0x07,0x10,0x07,0x08,0x07,0x00])
    spi.xfer3([0x08,0x00,0x08,0x20,0x08,0x04,0x08,0x00])

    time.sleep(1.0)

    spi.xfer3([0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00])
    spi.xfer3([0x02,0x00,0x02,0x03,0x02,0xc0,0x02,0x00])
    spi.xfer3([0x03,0x00,0x03,0x02,0x03,0x40,0x03,0x00])
    spi.xfer3([0x04,0x00,0x04,0x03,0x04,0xc0,0x04,0x00])
    spi.xfer3([0x05,0x00,0x05,0x00,0x05,0x00,0x05,0x00])
    spi.xfer3([0x06,0x00,0x06,0x10,0x06,0x08,0x06,0x00])
    spi.xfer3([0x07,0x00,0x07,0x10,0x07,0x08,0x07,0x00])
    spi.xfer3([0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00])
