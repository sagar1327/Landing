#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv
from matplotlib import pyplot as plt


class BitWis:
    """Masking and adding one image over another."""
    def __init__(self):
        self.img1 = cv.imread("logo.jpg")
        # self.re_img1 = cv.resize(self.img1, None, fx=0.1, fy=0.1, interpolation=cv.INTER_AREA)
        self.img2 = cv.imread("python.png")
        self.re_img2 = cv.resize(self.img2, (1000, 1000), interpolation=cv.INTER_AREA)
        self.masking()

    def masking(self):
        """Masking process."""
        # rows, cols, channels = self.re_img1.shape
        rows, cols, channels = self.img1.shape
        roi = self.re_img2[0:rows, 0:cols]

        # masking
        # gray = cv.cvtColor(self.re_img1, cv.COLOR_BGR2GRAY)
        gray = cv.cvtColor(self.img1, cv.COLOR_BGR2GRAY)
        ret, mask = cv.threshold(gray, 180, 255, cv.THRESH_BINARY)  # bg mask
        mask_inv = cv.bitwise_not(mask)  # fg mask

        img1_bg = cv.bitwise_and(roi, roi, mask=mask)
        # img1_fg = cv.bitwise_and(self.re_img1, self.re_img1, mask=mask)
        img1_fg = cv.bitwise_and(self.img1, self.img1, mask=mask_inv)

        comb = cv.add(img1_bg, img1_fg)
        self.re_img2[0:rows, 0:cols] = comb

        cv.imshow("Display", self.re_img2)
        cv.waitKey(0)
        cv.destroyAllWindows()


def main():
    BitWis()


if __name__ == "__main__":
    main()
