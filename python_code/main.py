import RPi.GPIO as GPIO
import time
import cv2

from rpi_camera import RpiCamera
from tennis_detect import TennisDetect
from smbus import SMBus

import spidev


GPIO.setwarnings(False)         # Disable warning
GPIO.setmode(GPIO.BCM)          # BCM coding

spi = spidev.SpiDev()           # spi for the LED monitor
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi_writen_flag = 0x00          # spi flag for the motion control

camera_center_x = 1080.0/2      # center of the frame
radius_target = 80.0            # the target radius used in the PID control

stm_main = 0x29                 # address of the motor driver
i2cbus = SMBus(1)               # i2c transmission
stm_sleep_time = 0.01           # sleep time for i2c
arm_status = 0x00               # status flag for motion control
robot_status = 0x00             # 0x00 means status 1
tennis_search_start_time = 0    # time flag for tennis searching
tennis_search_end_time = 0      # time flag for tennit searching

# initialize the LED monitor
spi.xfer2([0x09, 0x00])
spi.xfer2([0x0a, 0x04])
spi.xfer2([0x0b, 0x07])
spi.xfer2([0x0c, 0x01])
spi.xfer2([0x0f, 0x00])
spi.xfer2([0x00, 0x00])
spi.xfer2([0x00, 0x00])
spi.xfer2([0x00, 0x00])


class Robot(RpiCamera, TennisDetect):  # class robot
    def __init__(self):
        RpiCamera.__init__(self)
        TennisDetect.__init__(self)


if __name__ == '__main__':
    try:
        robot = Robot()

        # prepare for the transmission to pc
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
                frame_detect, x_pos, y_pos, radius, rate = robot.tennis_detect(
                    frame_origin, video_return)
                robot.video_transmission_to_pc(frame_detect)
                video_out.write(frame_detect)

            else:
                x_pos, y_pos, radius, rate = robot.tennis_detect(
                    frame_origin, video_return)

            if radius == 0:  # radius ==0 means it hasn't found the tennis
                pass
            else:
                # use the moving average to reduce the error
                radius_mov_ave = 0.4 * radius + 0.6 * radius_mov_ave
                x_mov_ave = 0.4 * x_pos + 0.6 * x_mov_ave

            rawCapture.truncate(0)

            mfps = 1 / (time.time() - t_start)  # FPS
            print('FPS: ', mfps, robot_status)

            # motion control
            if robot_status == 0x00:
                if spi_writen_flag == 0x00:
                    spi.xfer3([0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00])
                    spi.xfer3([0x02, 0x00, 0x02, 0x03, 0x02, 0xc0, 0x02, 0x00])
                    spi.xfer3([0x03, 0x00, 0x03, 0x02, 0x03, 0x40, 0x03, 0x00])
                    spi.xfer3([0x04, 0x00, 0x04, 0x03, 0x04, 0xc0, 0x04, 0x00])
                    spi.xfer3([0x05, 0x00, 0x05, 0x00, 0x05, 0x00, 0x05, 0x00])
                    spi.xfer3([0x06, 0x00, 0x06, 0x10, 0x06, 0x08, 0x06, 0x00])
                    spi.xfer3([0x07, 0x00, 0x07, 0x10, 0x07, 0x08, 0x07, 0x00])
                    spi.xfer3([0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00])
                    spi_writen_flag = 0x01

                error_x = camera_center_x - x_mov_ave
                error_l = radius_target - radius_mov_ave

                speed_r = 0.00023 * error_x
                speed_l = 0.013 * error_l

                if speed_l < -1.0:
                    speed_l = -1.0
                elif speed_l > 1.0:
                    speed_l = 1.0

                if speed_r < -0.1:
                    speed_r = -0.1
                elif speed_r > 0.1:
                    speed_r = 0.1

                if rate < 0.6:
                    speed_l = 0
                    speed_r = 0

                if abs(x_mov_ave - camera_center_x) > 200 or abs(radius_mov_ave - radius_target) > 20:
                    tennis_search_start_time = time.time()
                tennis_search_end_time = time.time()

                if tennis_search_end_time - tennis_search_start_time > 2.5:
                    robot_status = 0x01
                    tennis_search_start_time = time.time()
                    spi_writen_flag = 0x00

            if robot_status == 0x01:
                if spi_writen_flag == 0x00:
                    spi.xfer3([0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00])
                    spi.xfer3([0x02, 0x00, 0x02, 0x03, 0x02, 0xc0, 0x02, 0x00])
                    spi.xfer3([0x03, 0x00, 0x03, 0x02, 0x03, 0x40, 0x03, 0x00])
                    spi.xfer3([0x04, 0x00, 0x04, 0x23, 0x04, 0xc4, 0x04, 0x00])
                    spi.xfer3([0x05, 0x00, 0x05, 0x10, 0x05, 0x08, 0x05, 0x00])
                    spi.xfer3([0x06, 0x00, 0x06, 0x08, 0x06, 0x10, 0x06, 0x00])
                    spi.xfer3([0x07, 0x00, 0x07, 0x10, 0x07, 0x08, 0x07, 0x00])
                    spi.xfer3([0x08, 0x00, 0x08, 0x20, 0x08, 0x04, 0x08, 0x00])
                    spi_writen_flag = 0x01

                speed_r = 0.0
                speed_l = 0.33
                arm_status = 0x00

                tennis_search_end_time = time.time()

                if tennis_search_end_time - tennis_search_start_time > 2.75:
                    robot_status = 0x02
                    tennis_search_start_time = time.time()
                    spi_writen_flag = 0x00

            if robot_status == 0x02:

                speed_r = 0.0
                speed_l = 0.00
                arm_status = 0x23

                if spi_writen_flag == 0x00:
                    spi.xfer3([0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00])
                    spi.xfer3([0x02, 0x00, 0x02, 0x01, 0x02, 0x80, 0x02, 0x00])
                    spi.xfer3([0x03, 0x00, 0x03, 0x02, 0x03, 0x40, 0x03, 0x00])
                    spi.xfer3([0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00])
                    spi.xfer3([0x05, 0x00, 0x05, 0x00, 0x05, 0x00, 0x05, 0x00])
                    spi.xfer3([0x06, 0x00, 0x06, 0x0a, 0x06, 0x50, 0x06, 0x00])
                    spi.xfer3([0x07, 0x00, 0x07, 0x04, 0x07, 0x20, 0x07, 0x00])
                    spi.xfer3([0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00])
                    spi_writen_flag = 0x01

                tennis_search_end_time = time.time()

                if tennis_search_end_time - tennis_search_start_time > 0.5:
                    robot_status = 0x03
                    tennis_search_start_time = time.time()
                    spi_writen_flag = 0x00

            if robot_status == 0x03:

                speed_r = 0.0
                speed_l = 0.00
                arm_status = 0x23

                tennis_search_end_time = time.time()

                if tennis_search_end_time - tennis_search_start_time > 0.5:
                    robot_status = 0x04
                    tennis_search_start_time = time.time()
                    spi_writen_flag = 0x00

            if robot_status == 0x04:

                speed_r = 0.2
                speed_l = 0.0
                arm_status = 0x23

                tennis_search_end_time = time.time()

                if tennis_search_end_time - tennis_search_start_time > 1.0:
                    robot_status = 0x05
                    tennis_search_start_time = time.time()
                    spi_writen_flag = 0x00

            if robot_status == 0x05:

                speed_r = 0.0
                speed_l = 0.4
                arm_status = 0x23

                tennis_search_end_time = time.time()

                if tennis_search_end_time - tennis_search_start_time > 3.0:
                    robot_status = 0x06
                    tennis_search_start_time = time.time()
                    spi_writen_flag = 0x00

            if robot_status == 0x06:

                speed_r = 0.0
                speed_l = 0.0
                arm_status = 0x00

                tennis_search_end_time = time.time()

                if tennis_search_end_time - tennis_search_start_time > 3.0:
                    robot_status = 0x07
                    tennis_search_start_time = time.time()
                    spi_writen_flag = 0x00

            # i2c transmission
            i2cbus.write_byte(stm_main, (int(127 * 0) + int(127)))
            time.sleep(stm_sleep_time)
            i2cbus.write_byte(stm_main, (int(127 * speed_l) + int(127)))
            time.sleep(stm_sleep_time)
            i2cbus.write_byte(stm_main, (int(127 * speed_r) + int(127)))
            time.sleep(stm_sleep_time)
            i2cbus.write_byte(stm_main, arm_status)

    except KeyboardInterrupt:
        print("stopped")
