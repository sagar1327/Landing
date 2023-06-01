import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


class Thresh:
    """Thresholding frames from voxl camera"""
    def __init__(self):
        img = cv.imread("tracking.jpg", 0)
        r, c = img.shape
        self.roi = img[60:r, 16:204]
        self.masking()

    def masking(self):
        thresh = cv.adaptiveThreshold(self.roi, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 19, 6)
        mask = cv.bitwise_not(thresh)
        plt.imshow(mask, 'gray')
        plt.show()


def main():
    Thresh()


if __name__ == "__main__":
    main()
