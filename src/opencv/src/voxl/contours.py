import cv2 as cv
import numpy as np


class Contours:
    def __init__(self):
        data = np.load("tracking_camera_intrinsic_data.npz")
        mtx = data['camera_matrix']
        dist = data['distortion_coefficient']
        cap = cv.VideoCapture("tracking_05_06_23.mp4")
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
        # circle = cv.circle(im2, (int(column / 2) - 9, rows + 20), 70, (30, 200, 140), -5)
        # circle_gray = cv.cvtColor(circle, cv.COLOR_BGR2GRAY)
        rx = int(column / 2) - 9
        ry = rows + 25
        n = 0
        # for i in range(column):
        #     for j in range(rows):
        #         if np.sqrt((rx-i)**2 + (ry-j)**2) > 80:
        #             im2[j, i] = [121, 121, 121]
        #
        roi = im2[60:rows, 84:144]
        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 0)
        ret3, otsu_thresh = cv.threshold(blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        otsu_mask = cv.bitwise_not(otsu_thresh)
        contours, hierarchy = cv.findContours(otsu_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        print(box)
        cv.drawContours(roi, [box], -1, (0, 0, 255), 1)
        # area = cv.contourArea(contours[0])
        # l, hr, hc = hierarchy.shape
        # area = []
        # # print(hierarchy)
        #
        # for i in range(hr):
        #     cont = contours[i]
        #     X, Y, W, H = cv.boundingRect(cont)
        #     area.append(W*H)
        # index = area.index(max(area))
        # cnt = contours[index]
        # leftmost = tuple(cnt[cnt[:, :, 0].argmin()][0])
        # rightmost = tuple(cnt[cnt[:, :, 0].argmax()][0])
        #
        # if 63 <= cnt[cnt[:, :, 1]][0] <= 68:
        #     print()
        # if 0 <= leftmost[0] <= 5 and 183 <= rightmost[0] <= 188:
        #     rect = cv.minAreaRect(cnt)
        #     box = cv.boxPoints(rect)
        #     box = np.int0(box)
        #     # # # print(box)
        #     cv.drawContours(roi2, [box], -1, (0, 0, 255), 1)
        #     # [vx, vy, x, y] = cv.fitLine(cont, cv.DIST_L2, 0, 0.01, 0.01)
        #     # # lefty = int((-x * vy / vx) + y)
        #     # # righty = int(((189 - x) * vy / vx) + y)
        #     # # cv.line(roi2, (189 - 1, righty), (0, lefty), (0, 255, 0), 2)
        #     #
        # cv.drawContours(roi, contours, 0, (255, 0, 0), 1)

        im2 = cv.resize(otsu_mask, None, fx=5, fy=5, interpolation=cv.INTER_AREA)
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
