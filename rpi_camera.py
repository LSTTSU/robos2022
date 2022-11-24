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

        host = '192.168.12.90'
        port = 1080
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.connect((host, port))
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32

    def rpi_camera_init(self):
        raw_capture = PiRGBArray(self.camera, size=(640, 480))
        raw_capture.truncate(0)
        time.sleep(2)

        return self.camera, raw_capture

    def video_transmission_to_pc(self, frame):
        result, img_encode = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 10])
        try:
            self.server.sendall(struct.pack('i', img_encode.shape[0]))
            self.server.sendall(img_encode)
            print("have sent one frame")
        except:
            print("fail to send the frame")

    def camera_cleanup(self):
        self.server.sendall(struct.pack('c', 1))
        self.server.close()
        self.camera.close()


if __name__ == '__main__':
    exit(1)
