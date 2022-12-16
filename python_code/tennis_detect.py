import cv2
import numpy as np
import copy


# import time
# import pdb


class TennisDetect(object):
    def __init__(self):
        #        self.lower_range = np.array([25, 0, 0])
        #       self.higher_range = np.array([50, 255, 255])
        self.lower_range = np.array([15, 100, 100])
        self.higher_range = np.array([150, 255, 255])
        self.cut_edge = 260
        pass

    def tennis_detect(self, img, video_return):
        x_pos = 0
        y_pos = 0
        radius = 0
        img_out = copy.copy(img)
        img = img[self.cut_edge:479, :, :]
        img = cv2.blur(img, (5, 5))
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # rgb to HSV

        # img_out = copy.copy(hsv_img[:, :, 0])
        # img_out = cv2.cvtColor(img_out, cv2.COLOR_GRAY2BGR)

        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # rgb to gray

        circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 0.25, 300, param1=100, param2=20, minRadius=25,
                                   maxRadius=100)

        if circles is not None:
            x = circles[0][:, 0].astype(int)  # extract the x, y, r of all detected circles
            y = circles[0][:, 1].astype(int)
            r = circles[0][:, 2].astype(int)
            s_r = (r / 1.5).astype(int)

            num = circles[0].shape[0]
            rate = np.zeros(num)

            for i in range(num):  # traverse all detected circles
                detect_area = (
                    hsv_img[y[i] - s_r[i]: y[i] + s_r[i],
                    x[i] - s_r[i]:x[i] + s_r[i]])  # A square in the detected circle
                height, width, channel = detect_area.shape

                if height != 0 and width != 0:
                    tennis_color_mask = cv2.inRange(detect_area, self.lower_range, self.higher_range)
                    num_point = np.sum(tennis_color_mask / 255)
                    rate[i] = num_point / (height * width)
                    img_out = cv2.circle(img_out, (x[i], y[i] + self.cut_edge), r[i], (0, 255, 0), thickness=2)

            i = np.argmax(rate)  # select the circle with the maximum rate as the detected tennis
            if rate[i] > 0.4:  # if the percent of tennis_color_point in detect_area > 50%, then regard it as the tennis
                x_pos = x[i]
                y_pos = y[i]
                radius = r[i]
            print('x: ', x[i], '  y: ', y[i], '  r: ', r[i], '   rate: ', rate[i])

        if video_return:  # if it needs to return the frame with the detected tennis
            img_out = cv2.circle(img_out, (x_pos, y_pos + self.cut_edge), radius, (0, 0, 255), thickness=10)
            return img_out, x_pos, y_pos, radius
        else:  # if it only needs to return the position of the detected tennis
            return x_pos, y_pos, radius


if __name__ == '__main__':
    try:
        print('This file should be imported')

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
