#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv
from matplotlib import pyplot as plt

img = cv.imread("img_1745.jpg")
img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)  # only accept grayscale images
img_new = cv.adaptiveThreshold(img, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 11, 2)
img_new2 = cv.adaptiveThreshold(img, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)
images = [img, img_new, img, img_new2]
for i in range(4):
    plt.subplot(2, 2, i+1)
    plt.imshow(images[i], 'gray')
plt.show()
