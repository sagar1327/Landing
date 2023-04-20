#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv
from matplotlib import pyplot as plt

img = cv.imread("img_1745.jpg", 0)  # only accept grayscale images
img_new = cv.threshold(img, 0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)

images = [img, img_new]
cv.imshow("Display", img_new)
cv.waitKey(0)
# for i in range(2):
#     plt.subplot(1, 2, i+1)
#     plt.imshow(images[i], 'gray')
# plt.show()
