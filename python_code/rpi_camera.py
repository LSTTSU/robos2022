# import time
import numpy as np
import cv2
import socket
import struct
import time

from picamera.array import PiRGBArray
from picamera import PiCamera


class RpiCamera(object):
    def __init__(self):

        host = '192.168.0.25'
        port = 1080

        self.transmission = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.transmission.connect((host, port))

        self.rpi_camera = PiCamera()
        self.rpi_camera.resolution = (1080,480)
        self.rpi_camera.framerate = 32

    def rpi_camera_init(self):
        time.sleep(0.5)
        original_img = PiRGBArray(self.rpi_camera, size=(1080,480))
        original_img.truncate(0)
        time.sleep(3)

        return self.rpi_camera, original_img

    def video_transmission_to_pc(self, frame):
        result, img_encode = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 10])
        try:
            self.transmission.sendall(struct.pack('i', img_encode.shape[0]))
            self.transmission.sendall(img_encode)
            print("data sent")
        except:
            print("fail")

    def camera_cleanup(self):
        self.transmission.sendall(struct.pack('c', 1))
        self.transmission.close()
        self.rpi_camera.close()


if __name__ == '__main__':
    exit(1)
