import cv2 as cv
import numpy as np

# params for ShiTomasi corner detection
feature_params = dict(maxCorners=100,
                      qualityLevel=0.9,
                      minDistance=2,
                      blockSize=2)
# Parameters for lucas kanade optical flow
lk_params = dict(winSize=(15, 15),
                 maxLevel=15,
                 criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 30, 0.1))
# Create some random colors
color = np.random.randint(0, 255, (100, 3))


class Contours:
    def __init__(self):
        data = np.load("tracking_camera_intrinsic_data.npz")
        self.mtx = data['camera_matrix']
        self.dist = data['distortion_coefficient']
        self.cap = cv.VideoCapture("tracking_05_06_23.mp4")
        if not self.cap.isOpened():
            print("Can't open video file")
            exit()

        self.feature()

    def feature(self):
        ret, old_frame = self.cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            exit()
        h, w = old_frame.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1)
        # undistort
        mapx, mapy = cv.initUndistortRectifyMap(self.mtx, self.dist, None, newcameramtx, (w, h), 5)
        dst = cv.remap(old_frame, mapx, mapy, cv.INTER_LINEAR)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]

        rows, column, channel = dst.shape
        roi = dst[75:rows, 30:196]
        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 0)
        ret3, old_otsu_thresh = cv.threshold(blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        old_otsu_mask = cv.bitwise_not(old_otsu_thresh)
        mask = np.zeros_like(roi)
        p0 = cv.goodFeaturesToTrack(old_otsu_mask, mask=None, **feature_params)

        while True:
            ret, new_frame = self.cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                exit()
            h, w = new_frame.shape[:2]
            newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1)
            mapx, mapy = cv.initUndistortRectifyMap(self.mtx, self.dist, None, newcameramtx, (w, h), 5)
            dst = cv.remap(new_frame, mapx, mapy, cv.INTER_LINEAR)
            # crop the image
            x, y, w, h = roi
            dst = dst[y:y + h, x:x + w]

            rows, column, channel = dst.shape
            roi = dst[75:rows, 30:196]
            gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
            blur = cv.GaussianBlur(gray, (5, 5), 0)
            ret3, new_otsu_thresh = cv.threshold(blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
            p1, st, err = cv.calcOpticalFlowPyrLK(old_otsu_mask, new_otsu_thresh, p0, None, **lk_params)

            if p1 is not None:
                good_new = p1[st == 1]
                good_old = p0[st == 1]
            else:
                break

            # draw the tracks
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                mask = cv.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
                roi = cv.circle(roi, (int(a), int(b)), 5, color[i].tolist(), -1)
            img = cv.add(roi, mask)

            im = cv.resize(img, None, fx=5, fy=5, interpolation=cv.INTER_AREA)
            cv.imshow('frame', im)
            if cv.waitKey(0) == ord('q'):
                break

        cv.destroyAllWindows()


def main():
    Contours()


if __name__ == "__main__":
    main()
