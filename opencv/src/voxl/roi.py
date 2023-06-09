import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


class Contours:
    def __init__(self):
        data = np.load("tracking_camera_intrinsic_data.npz")
        self.mtx = data['camera_matrix']
        self.dist = data['distortion_coefficient']
        self.cap = cv.VideoCapture("tracking_05_06_23.mp4")
        if not self.cap.isOpened():
            print("Can't open video file")
            exit()

    def undistort(self, img):
        h, w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1)
        # undistort
        mapx, mapy = cv.initUndistortRectifyMap(self.mtx, self.dist, None, newcameramtx, (w, h), 5)
        dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        return dst

    def temp_matching(self):
        ret1, frame1 = self.cap.read()
        if not ret1:
            print("Did the video end?")
            exit()
        un_frame = self.undistort(frame1)
        rows, columns, channel = un_frame.shape
        template = un_frame[75:rows, 38:196]
        w, h = template.shape[0:2]
        while True:
            ret2, frame2 = self.cap.read()
            if not ret2:
                print("Did the video end?")
                exit()
            un_frame2 = self.undistort(frame2)
            method = eval('cv.TM_SQDIFF')
            # Apply template Matching
            res = cv.matchTemplate(un_frame2, template, method)
            min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
            top_left = min_loc
            bottom_right = (top_left[0] + h, top_left[1] + w)
            cv.rectangle(un_frame2, top_left, bottom_right, 255, 2)
            cv.imshow("Display", un_frame2)
            if cv.waitKey(0) == ord('q'):
                break
        cv.destroyAllWindows()


def main():
    ct = Contours()
    ct.temp_matching()


if __name__ == "__main__":
    main()
