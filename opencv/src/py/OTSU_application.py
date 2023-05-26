#!/home/sagar/ROS/opencv/env/bin/python

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

try:
    img = cv.imread("frame0001.jpg", 0)
    # roi
    row, column = img.shape
    roi = img[137:row, 0:column]
    # Normal thresholding
    ret1, img_new1 = cv.threshold(roi, 100, 255, cv.THRESH_BINARY)
    maks1 = cv.bitwise_not(img_new1)
    # Adaptive thresholding
    img_new2 = cv.adaptiveThreshold(roi, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 11, 2)
    mask2 = cv.bitwise_not(img_new2)
    laplacian = cv.Laplacian(roi, ddepth=-1, ksize=5)
    edge = cv.Canny(roi, 40, 255)
    # kernel = np.array([[0, 0, 0],
    #                    [0, 1, 0],
    #                    [0, 0, 1]], np.uint8)
    # dilate = cv.dilate(mask2, kernel, iterations=1)
    # closing = cv.morphologyEx(mask2, cv.MORPH_CLOSE, kernel)
    # OTSU thresholding
    ret2, img_new3 = cv.threshold(roi, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    mask3 = cv.bitwise_not(img_new3)

    image = [roi, maks1, mask2, laplacian, edge, mask3]
    for i in range(6):
        plt.subplot(3, 3, i + 1)
        plt.imshow(image[i], 'gray')
    plt.show()
except cv.error:
    print("Can't read image file")