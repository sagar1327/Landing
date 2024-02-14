import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


class Features:
    def __init__(self):
        data = np.load("tracking_camera_intrinsic_data.npz")
        self.mtx = data['camera_matrix']
        self.dist = data['distortion_coefficient']
        self.cap = cv.VideoCapture("whiteboard.mp4")
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
        # The shape size of the original image is 640x480, and the size of undistorted image is 244x128.
        # The Aspect ratio needs to be fixed
        dst = cv.resize(dst, None, fx=2.62295, fy=3.75, interpolation=cv.INTER_AREA)
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
                print("Video end.")
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

            # BFMatcher with default params
            bf = cv.BFMatcher()
            matches = bf.knnMatch(old_des, new_des, k=2)

            # Apply ratio test
            old_good = []
            for m, n in matches:
                if m.distance < 0.4 * n.distance:
                    old_good.append([m])

            # cv.drawMatchesKnn expects list of lists as matches.
            img3 = cv.drawMatchesKnn(old_un_frame, old_kp, new_un_frame, new_kp, old_good, None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            old_un_frame = new_un_frame.copy()
            old_kp = new_kp
            old_des = new_des

            old_pts = np.float32([kp.pt for kp in old_kp]).reshape(-1, 1, 2)
            old_matched_pt = np.zeros((len(old_good), 2))
            for i in range(len(old_good)):
                index1 = old_good[i][0].queryIdx
                old_matched_pt[i, :] = old_pts[index1][0]

            # img3 = cv.resize(img3, None, fx=2, fy=2, interpolation=cv.INTER_AREA)
            cv.imshow("Display", img3)
            if cv.waitKey(0) == ord('q'):
                break
            elif cv.waitKey(0) == ord('s'):
                cv.imwrite("feature 3.png", img3)
                break
        cv.destroyAllWindows()

    def contour(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Video ended.")
                exit()
            un_frame = self.undistort(frame)
            gray = cv.cvtColor(un_frame, cv.COLOR_BGR2GRAY)
            ret, otsu_thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
            otsu_mask = cv.bitwise_not(otsu_thresh)
            row, column = otsu_mask.shape
            corners = cv.goodFeaturesToTrack(otsu_mask, 95, 0.1, 10)
            corners = np.int0(corners)

            for i in corners:
                x, y = i.ravel()
                # print("x: {} and y: {}".format(x, y))
                cv.circle(un_frame, (x, y), 3, 255, -1)

            # contours, hierarchy = cv.findContours(otsu_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            # cnt = contours[1]
            # rect = cv.minAreaRect(cnt)
            # box = cv.boxPoints(rect)
            # box = np.int0(box)
            # print(box)
            # cv.drawContours(roi, [box], -1, (0, 0, 255), 1)
            # area = cv.contourArea(contours[0])
            # l, hr, hc = hierarchy.shape
            # area = []
            # print(contours)
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
            # cv.drawContours(un_frame, contours, -1, (255, 0, 0), 1)

            # images = [img, gray, blur, edge]
            # for i in range(4):
            #     plt.subplot(2, 2, i + 1)
            #     im = cv.cvtColor(images[i], cv.COLOR_BGR2RGB)
            #     plt.imshow(im)
            # plt.show()
            cv.imshow("Display", un_frame)
            if cv.waitKey(0) == ord('q'):
                break
            elif cv.waitKey(0) == ord('s'):
                cv.imwrite("feature 3.png",un_frame)
                break
        cv.destroyAllWindows()


def main():
    ct = Features()
    # ct.find_obj()
    # ct.temp_matching()
    ct.contour()


if __name__ == "__main__":
    main()
