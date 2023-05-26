#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv
import numpy
import matplotlib.pyplot as plt

img1 = cv.imread("python.png")  # size 3334x3334
img2 = cv.imread("OpenCV.png")  # size 831x1024

# image resizing
r_img2 = cv.resize(img1, (831, 1024), interpolation=cv.INTER_AREA)
# blending, require images to be of same size and number of channel
blend_img = cv.addWeighted(img2, 0.5, r_img2, 0.5, 0)

cv.imshow("Display", blend_img)
cv.waitKey(0)
cv.destroyAllWindows()

# OR plotting using matplotlib
# color correction, since opencv uses bgr format
# img1 = cv.cvtColor(img1, cv.COLOR_BGR2RGB)
# img2 = cv.cvtColor(img2, cv.COLOR_BGR2RGB)
# img = [img1, img2, r_img2, blend_img]
# for i in range(4):
#     plt.subplot(2, 2, i+1)
#     plt.imshow(img[i])
# plt.show()
