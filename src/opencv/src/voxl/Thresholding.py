import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


class Thresh:
    """Thresholding frames from voxl camera"""
    def __init__(self):
        img = cv.imread("U1.png", 0)
        self.img = img
        r, c = img.shape
        # self.roi = img[60:r, 16:204]
        self.masking()

    def masking(self):
        ret, thresh = cv.threshold(self.img, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        mask = cv.bitwise_not(thresh)

        img = [self.img, mask]
        for i in range(0, 2):
            plt.subplot(1, 2, i+1)
            plt.imshow(img[i], 'gray')
        plt.show()


def main():
    Thresh()


if __name__ == "__main__":
    main()
