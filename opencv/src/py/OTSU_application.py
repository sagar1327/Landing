#!/home/sagar/ROS/opencv/env/bin/python

import cv2 as cv
from matplotlib import pyplot as plt


class OTSU:
    """Application of OTSU thresholding"""
    def __init__(self):
        try:
            img = cv.imread("img_1745.jpg", 0)
            # OTSU thresholding
            ret, thresh = cv.threshold(img, 0, 200, cv.THRESH_BINARY + cv.THRESH_OTSU)
            plt.imshow(thresh, 'gray')
            plt.show()
        except cv.error:
            print("Can't read image file")


def main():
    OTSU()


if __name__ == '__main__':
    main()
