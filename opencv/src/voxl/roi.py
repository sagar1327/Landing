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
        roi1 = un_frame[75:rows, 30:203]
        gray = cv.cvtColor(roi1, cv.COLOR_BGR2GRAY)
        ret, otsu_thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        otsu_mask = cv.bitwise_not(otsu_thresh)
        kernel = np.ones((7, 7), np.uint8)
        closing = cv.morphologyEx(otsu_mask, cv.MORPH_CLOSE, kernel)
        # opening = cv.morphologyEx(closing, cv.MORPH_OPEN, kernel)
        template = cv.bitwise_and(roi1, roi1, mask=closing)
        w, h = template.shape[0:2]
        while True:
            ret, frame2 = self.cap.read()
            if not ret:
                print("Did the video end?")
                exit()
            un_frame2 = self.undistort(frame2)
            meth = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
                    'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']
            method = eval(meth[5])
            # Apply template Matching
            res = cv.matchTemplate(un_frame2, template, method)
            min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
            if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
                top_left = min_loc
            else:
                top_left = max_loc
            bottom_right = (top_left[0] + h, top_left[1] + w)
            cv.rectangle(un_frame2, top_left, bottom_right, 255, 2)
            cv.imshow("Display", un_frame2)
            if cv.waitKey(0) == ord('q'):
                cv.imwrite('zoomed_out.jpg', un_frame2)
                break
        cv.destroyAllWindows()

    def find_obj(self):
        ret, old_frame = self.cap.read()
        if not ret:
            print("Video ended.")
            exit()
        old_un_frame = self.undistort(old_frame)
        old_gray = cv.cvtColor(old_un_frame, cv.COLOR_BGR2GRAY)
        ret, old_otsu_thresh = cv.threshold(old_gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        old_otsu_mask = cv.bitwise_not(old_otsu_thresh)
        kernel = np.ones((7, 7), np.uint8)
        old_closing = cv.morphologyEx(old_otsu_mask, cv.MORPH_CLOSE, kernel)

        # SIFT (Scale-Invariant Feature Transform)
        sift = cv.SIFT_create()
        old_kp, old_des = sift.detectAndCompute(old_closing, None)

        # ORB (Oriented FAST and Rotated BRIEF)
        # orb = cv.ORB_create()
        # old_kp = orb.detect(old_closing, None)
        # old_kp, old_des = orb.compute(old_closing, old_kp)

        while True:
            ret, new_frame = self.cap.read()
            if not ret:
                print("Video ended.")
                exit()
            new_un_frame = self.undistort(new_frame)
            new_gray = cv.cvtColor(new_un_frame, cv.COLOR_BGR2GRAY)
            ret, new_otsu_thresh = cv.threshold(new_gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
            new_otsu_mask = cv.bitwise_not(new_otsu_thresh)
            kernel = np.ones((7, 7), np.uint8)
            new_closing = cv.morphologyEx(new_otsu_mask, cv.MORPH_CLOSE, kernel)
            new_kp, new_des = sift.detectAndCompute(new_closing, None)
            # new_kp = orb.detect(new_closing, None)
            # new_kp, new_des = orb.compute(new_closing, new_kp)

            # BFMatcher with default params
            bf = cv.BFMatcher()
            matches = bf.knnMatch(old_des, new_des, k=2)

            # Apply ratio test
            good = []
            for m, n in matches:
                if m.distance < 0.4 * n.distance:
                    good.append([m])

            # cv.drawMatchesKnn expects list of lists as matches.
            img3 = cv.drawMatchesKnn(old_un_frame, old_kp, new_un_frame, new_kp, good, None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            # old_un_frame = new_un_frame.copy()
            # old_kp = new_kp
            # old_des = new_des
            # plt.imshow(img3), plt.show()
            print(old_des[0])

            img3 = cv.resize(img3, None, fx=2, fy=2, interpolation=cv.INTER_AREA)
            cv.imshow("Display", img3)
            if cv.waitKey(0) == ord('q'):
                break
        cv.destroyAllWindows()


def main():
    ct = Contours()
    # ct.temp_matching()
    ct.find_obj()


if __name__ == "__main__":
    main()
