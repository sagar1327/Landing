#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


class BitWis:
    """Masking and adding one image over another."""
    def __init__(self):
        try:
            # self.img = cv.imread("frame0001.jpg", 0)
            cap = cv.VideoCapture("tracking.mp4")
            if not cap.isOpened():
                print("Cannot open camera")
                exit()
            while True:
                # Capture frame
                ret, frame = cap.read()
                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                if not ret:
                    print("Can't receive frame (stream end?). Exiting ...")
                    exit()
                row, column = gray.shape
                self.roi = gray[137:row, 0:column]
                self.masking()
                if cv.waitKey(0) == ord('q'):
                    break
        except cv.error:
            print("Did video end?")
            cv.destroyAllWindows()

    def masking(self):
        """Masking process."""
        # ret, thresh = cv.threshold(self.img, 59, 255, cv.THRESH_BINARY)  # bg mask
        ret, thresh = cv.threshold(self.roi, 50, 255, cv.THRESH_BINARY)  # bg mask
        laplacian = cv.Laplacian(self.roi, ddepth=-1, ksize=5)
        edge = cv.Canny(self.roi, 40, 255)
        mask = cv.bitwise_not(thresh)
        # kernel1 = np.array([[0, 0, 0],
        #                     [0, 1, 0],
        #                     [0, 0, 1]], np.uint8)
        # dilate = cv.dilate(mask, kernel1, iterations=7)
        # kernel2 = np.array([[0, 1, 0],
        #                     [1, 0, 1],
        #                     [0, 1, 0]], np.uint8)
        # erode = cv.erode(dilate, kernel2, iterations=11)
        # kernel3 = np.ones((3, 3), np.uint8)
        # closing = cv.morphologyEx(erode, cv.MORPH_CLOSE, kernel3)
        img_fg = cv.bitwise_and(self.roi, self.roi, mask=mask)
        img_fg = cv.cvtColor(img_fg, cv.COLOR_BGR2RGB)
        # img_fg = cv.bitwise_and(self.gray, self.gray, mask=closing)
        # img_fg = cv.cvtColor(img_fg, cv.COLOR_BGR2RGB)

        # img = [self.roi, mask, img_fg]
        # for i in range(3):
        #     plt.subplot(1, 3, i+1)
        #     plt.imshow(img[i], 'gray')
        # plt.show()

        cv.imshow("Display", thresh)


def main():
    BitWis()


if __name__ == "__main__":
    main()
