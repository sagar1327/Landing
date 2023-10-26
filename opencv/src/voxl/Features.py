import cv2 as cv
import numpy as np

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
        """Undistorting function"""
        H, W = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (W, H), 1)
        # undistort
        mapx, mapy = cv.initUndistortRectifyMap(self.mtx, self.dist, None, newcameramtx, (W, H), 5)
        dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        # The shape size of the original image is 640x480, and the size of undistorted image is 244x128.
        # The Aspect ratio needs to be fixed
        dst = cv.resize(dst, None, fx=W/w, fy=H/h, interpolation=cv.INTER_AREA)
        return dst

    def shadow_extraction(self):
        """Isolating the target shadow"""
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Video end.")
                exit()
            # Undistort the original frame
            un_frame = self.undistort(frame)
            gray = cv.cvtColor(un_frame, cv.COLOR_BGR2GRAY)
            ret, otsu_thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
            # Creating a mask
            otsu_mask = cv.bitwise_not(otsu_thresh)

            # Using contour method
            contours, hierarchy = cv.findContours(otsu_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            # print(list(contours))
            # Counter for possible contours
            count = 0
            # Getting the centroid
            for i in range(0, len(contours)):
                cnt = contours[i]
                M = cv.moments(cnt)
                if M['m00'] == 0:
                    pass
                else:
                    cx = M['m10'] / M['m00']
                    cy = M['m01'] / M['m00']
                    # Range of cx and cy
                    if 100 >= int(cy) or int(cy) >= 240 or 290 >= int(cx) or int(cx) >= 305:
                        # deleting unwanted hierarchy
                        hierarchy = np.delete(hierarchy, i-count, 1)
                        count += 1

            cv.imshow("Display", un_frame)
            if cv.waitKey(0) == ord('q'):
                break
        cv.destroyAllWindows()


def main():
    ct = Features()
    ct.shadow_extraction()


if __name__ == "__main__":
    main()

