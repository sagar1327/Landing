#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv
from matplotlib import pyplot as plt


class BitWis:
    """Masking and adding one image over another."""
    def __init__(self):
        self.img = cv.imread("test_img.jpg")
        self.masking()

    def masking(self):
        """Masking process."""
        gray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
        ret, mask = cv.threshold(gray, 140, 255, cv.THRESH_BINARY)  # bg mask
        mask_inv = cv.bitwise_not(mask)  # fg mask

        img_fg = cv.bitwise_and(gray, gray, mask=mask_inv)
        # img_fg = cv.cvtColor(img_fg, cv.COLOR_BGR2RGB)

        cv.imshow("Display", mask_inv)
        cv.waitKey(0)
        cv.destroyAllWindows()


def main():
    BitWis()


if __name__ == "__main__":
    main()
