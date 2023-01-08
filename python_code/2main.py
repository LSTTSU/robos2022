# this is a sub file of main.py

import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import time

from picamera.array import PiRGBArray
from picamera import PiCamera

from rpi_camera import RpiCamera
from tennis_detect import TennisDetect
from smbus import SMBus

GPIO.setwarnings(False)  # Disable warning
GPIO.setmode(GPIO.BCM)  # BCM coding

camera_center_x = 1080.0/2      # center of the frame used as x axis target
radius_target = 80.0

stm_main = 0x29
i2cbus = SMBus(1)
stm_sleep_time = 0.01
arm_status = 0x00
robot_status = 0x00             #  0x00 means status 1
ball_search_start_time = 0
ball_search_end_time = 0

class Robot(RpiCamera, TennisDetect):
    def __init__(self):
        RpiCamera.__init__(self)
        TennisDetect.__init__(self)


if __name__ == '__main__':
    try:
        robot = Robot()

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_out = cv2.VideoWriter('out.mp4', fourcc, 10, (720, 480))

        video_return = False
        radius_mov_ave = 35
        x_mov_ave = 320

        camera, rawCapture = robot.rpi_camera_init()  # Initialize the PiCamera

        for raw_frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            t_start = time.time()  # FPS

            frame_origin = raw_frame.array

            if video_return:
                frame_detect, x_pos, y_pos, radius, rate = robot.tennis_detect(frame_origin, video_return)
                robot.video_transmission_to_pc(frame_detect)
                video_out.write(frame_detect)
            else:
                x_pos, y_pos, radius, rate = robot.tennis_detect(frame_origin, video_return)

            # print('x: ', x_pos ,'  y: ', y_pos, '  r: ', radius)

            if radius == 0:  # radius ==0 means it hasn't found the tennis
                pass
            else:
                radius_mov_ave = 0.4 * radius + 0.6 * radius_mov_ave  # use the moving average  to reduce the error
                x_mov_ave = 0.4 * x_pos + 0.6 * x_mov_ave

            rawCapture.truncate(0)

            mfps = 1 / (time.time() - t_start)  # FPS
            print('FPS: ', mfps)

            if robot_status == 0x00:

                error_x = camera_center_x - x_mov_ave
                error_l = radius_target - radius_mov_ave

                speed_r = 0.0002 * error_x
                # speed_r = 0.0
                speed_l =  0.013 * error_l
                # speed_l =  -0.2

                if speed_l < -1.0:
                    speed_l = -1.0
                elif speed_l > 1.0:
                    speed_l = 1.0

                if speed_r < -0.1:
                    speed_r = -0.1
                elif speed_r > 0.1:
                    speed_l = 0.1

                if abs(x_mov_ave - camera_center_x) > 50 or abs(radius_mov_ave - radius_target) > 15:
                    ball_search_start_time = time.time()
                ball_search_end_time = time.time();

                if ball_search_end_time - ball_search_start_time > 100.0:
                    robot_status = 0x01;
                # if error_x < -25 or error_x > 25:
                #     speed_l = 0
                # else:
                #     speed_r = 0

            if robot_status == 0x01:

                speed_r = 0.0
                speed_l = 0.25
                arm_status = 0x00
                time.sleep(3.0)
                speed_r = 0.0
                speed_l = 0.0
                arm_status = 0x23
                time.sleep(3.0)

                robot_status = 0x02;

            if robot_status == 0x02:

                speed_r = 0.0
                speed_l = 0.00
                arm_status = 0x00

            i2cbus.write_byte(stm_main, (int(127 * 0) + int(127)))
            time.sleep(stm_sleep_time)
            i2cbus.write_byte(stm_main, (int(127 * speed_l) + int(127)))
            time.sleep(stm_sleep_time)
            i2cbus.write_byte(stm_main, (int(127 * speed_r) + int(127)))
            time.sleep(stm_sleep_time)
            i2cbus.write_byte(stm_main,arm_status)

    except KeyboardInterrupt:
        # i2cbus.write_byte(stm_main,0x00)
        # time.sleep(stm_sleep_time)
        # i2cbus.write_byte(stm_main,0x00)
        # time.sleep(stm_sleep_time)
        # i2cbus.write_byte(stm_main,0x00)
        # time.sleep(stm_sleep_time)
        # i2cbus.write_byte(stm_main,0x00)
        # time.sleep(0.5)
        print("stopped")
        # video_out.release()
