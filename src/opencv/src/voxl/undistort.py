import cv2 as cv
import numpy as np


class Undistort:
    """Correcting fisheye images."""
    def __init__(self):
        data = np.load("tracking_camera_intrinsic_data.npz")
        mtx = data['camera_matrix']
        dist = data['distortion_coefficient']
        # img = cv.imread("frame0198.jpg")
        cap = cv.VideoCapture("clip.mp4")
        if not cap.isOpened():
            print("Can't open video file")
            exit()
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Did video stream end?")
                break
            H, W = frame.shape[:2]
            newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (W, H), 1)
            # undistort
            mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (W, H), 5)
            dst = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)
            # crop the image
            x, y, w, h = roi
            dst = dst[y:y + h, x:x + w]
            dst = cv.resize(dst, None, fx=W/w, fy=H/h, interpolation=cv.INTER_AREA)

            cv.imshow('Display', dst)
            if cv.waitKey(0) == ord('q'):
                break
        cv.destroyAllWindows()


def main():
    Undistort()


if __name__ == "__main__":
    main()
