import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


class Contours:
    def __init__(self):
        data = np.load("tracking_camera_intrinsic_data.npz")
        mtx = data['camera_matrix']
        dist = data['distortion_coefficient']
        cap = cv.VideoCapture("tracking1.mp4")
        if not cap.isOpened():
            print("Can't open video file")
            exit()
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Did video stream end?")
                break
            h, w = frame.shape[:2]
            newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1)
            # undistort
            mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
            dst = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)
            # crop the image
            x, y, w, h = roi
            self.dst = dst[y:y + h, x:x + w]

            im1 = self.cnt()
            cv.imshow('Display', im1)
            if cv.waitKey(0) == ord('q'):
                break

    def cnt(self):
        im2 = self.dst
        rows, column, channel = im2.shape
        roi2 = im2[60:rows, 16:204]
        gray = cv.cvtColor(roi2, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 0)
        edge = cv.Canny(blur, 30, 255)
        # ret2, thresh = cv.threshold(gray, 35, 255, cv.THRESH_BINARY)
        # mask = cv.bitwise_not(thresh)
        # img_fg = cv.add(edge, mask)
        #
        # adap_thresh = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 19, 6)
        # adap_mask = cv.bitwise_not(thresh)
        # ret3, otsu_thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        # otsu_mask = cv.bitwise_not(thresh)
        contours, hierarchy = cv.findContours(edge, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        l, hr, hc = hierarchy.shape
        area = []
        # print(hierarchy)

        for i in range(hr):
            cont = contours[i]
            X, Y, W, H = cv.boundingRect(cont)
            area.append(W*H)
        index = area.index(max(area))
        rect = cv.minAreaRect(contours[index])
        box = cv.boxPoints(rect)
        box = np.int0(box)
        # # # print(box)
        cv.drawContours(roi2, [box], -1, (0, 0, 255), 1)
        # [vx, vy, x, y] = cv.fitLine(cont, cv.DIST_L2, 0, 0.01, 0.01)
        # # lefty = int((-x * vy / vx) + y)
        # # righty = int(((189 - x) * vy / vx) + y)
        # # cv.line(roi2, (189 - 1, righty), (0, lefty), (0, 255, 0), 2)
        #
        cv.drawContours(roi2, contours, index, (255, 255, 0), 1)
        im2 = cv.resize(roi2, None, fx=5, fy=5, interpolation=cv.INTER_AREA)
        return im2

        # images = [img, gray, blur, edge]
        # for i in range(4):
        #     plt.subplot(2, 2, i + 1)
        #     im = cv.cvtColor(images[i], cv.COLOR_BGR2RGB)
        #     plt.imshow(im)
        # plt.show()


def main():
    Contours()


if __name__ == "__main__":
    main()
