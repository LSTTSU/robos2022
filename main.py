import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import time

from picamera.array import PiRGBArray
from picamera import Pi22Camera

from rpi_camera import RpiCamera
from tennis_detect import TennisDetect
from smbus import SMBus

GPIO.setwarnings(False)  # Disable warning
GPIO.setmode(GPIO.BCM)  # BCM coding

stm_main = 0x29
i2cbus = SMBus(1)
stm_sleep_time = 0.01


class Robot(RpiCamera, TennisDetect):
    def __init__(self):
        RpiCamera.__init__(self)
        TennisDetect.__init__(self)


if __name__ == '__main__':
    try:
        robot = Robot()

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_out = cv2.VideoWriter('out.mp4', fourcc, 10, (720, 480))

        video_return = True
        radius_mov_ave = 35
        x_mov_ave = 320

        camera, rawCapture = robot.rpi_camera_init()  # Initialize the PiCamera

        for raw_frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            t_start = time.time()  # FPS

            frame_origin = raw_frame.array

            if video_return:
                frame_detect, x_pos, y_pos, radius = robot.tennis_detect(frame_origin, video_return)
                robot.video_transmission_to_pc(frame_detect)
                video_out.write(frame_detect)
            else:
                x_pos, y_pos, radius = robot.tennis_detect(frame_origin, video_return)
                # car.VideoTransmission(frame_origin)

            # print('x: ', x_pos ,'  y: ', y_pos, '  r: ', radius)

            if radius == 0:  # radius ==0 means it hasn't found the tennis
                pass
            else:
                radius_mov_ave = 0.4 * radius + 0.6 * radius_mov_ave  # use the moving average  to reduce the error
                x_mov_ave = 0.4 * x_pos + 0.6 * x_mov_ave

            rawCapture.truncate(0)

            mfps = 1 / (time.time() - t_start)  # FPS
            print('FPS: ', mfps)

            error_x = 0.0 - x_mov_ave
            error_l = 80.0 - radius_mov_ave

            speed_r = - 0.02 * error_x
            speed_l = - 0.02 * error_l

            if speed_l < -1.0:
                speed_l = -1.0
            elif speed_l > 1.0:
                speed_l = 1.0

            if speed_r < -1.0:
                speed_r = -1.0
            elif speed_r > 1.0:
                speed_l = 1.0

            if error_x < -10 or error_x > 10:
                speed_l = 0
            else:
                speed_r = 0

            i2cbus.write_byte(stm_main, 0x00)
            time.sleep(stm_sleep_time)
            i2cbus.write_byte(stm_main, hex(int(255 * speed_l / 1.0)))
            time.sleep(stm_sleep_time)
            i2cbus.write_byte(stm_main, hex(int(255 * speed_r / 1.0)))
            time.sleep(stm_sleep_time)
            i2cbus.write_byte(stm_main, 0x00)

    except KeyboardInterrupt:
        print("stopped")
        # video_out.release()