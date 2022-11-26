import cv2
import numpy
import socket
import struct

# import copy
# import numpy as np

HOST = '192.168.0.106'
PORT = 1080

buffSize = 65535

if __name__ == '__main__':
    host = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    host.bind((HOST, PORT))
    print('waiting for connect...')
    i = 7

    while True:
        data, address = host.recvfrom(buffSize)
        if len(data) == 1 and data[0] == 1:
            host.close()
            cv2.destroyAllWindows()
            exit(0)
        if len(data) != 4:
            length = 0
            continue
        else:
            length = struct.unpack('i', data)[0]
        data, address = host.recvfrom(buffSize)
        if length != len(data):
            print("check error")
            continue
        data = numpy.array(bytearray(data))
        img_decode = cv2.imdecode(data, 1)
        print('data received')

        cv2.imshow('datas', img_decode)

        key = cv2.waitKey(1)
        if key == ord('s'):
            cv2.imwrite('/photo_tennis/tennis_' + str(i) + '.jpg', img_decode)
            i = i + 1

        if cv2.waitKey(1) == 27:
            break

    host.close()
    cv2.destroyAllWindows()