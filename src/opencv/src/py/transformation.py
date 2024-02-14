#!/home/sagar/opencv_ws/envs/bin/python

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

# resize
img = cv.imread("python.png")
rows, col = img.shape[:2]
print('{}, {}'.format(rows, col))
img_new = cv.resize(img, (int(col/4), int(rows/4)), interpolation=cv.INTER_AREA)

# translation
M = np.float32([[1, 0, 100], [0, 1, 50]])
img_new2 = cv.warpAffine(img, M, (col, rows))

# rotation
M = cv.getRotationMatrix2D((col/2.0, rows/2.0), 90, 1)
img_new3 = cv.warpAffine(img, M, (col, rows))

# perspective transformation
# Method 1: Use three collinear points
img1 = cv.imread("sub.png")
# cv.imshow("Display", img1)
# cv.waitKey(0)
# rows, col = img1.shape[:2]
int_pts = np.float32([[714, 45], [962, 46], [600, 562]])
out_pts = np.float32([[0, 0], [1080, 0], [0, 1350]])
M = cv.getAffineTransform(int_pts, out_pts)
img_new4 = cv.warpAffine(img1, M, (1080, 1350))

# Method 2: Use four collinear points
int_pts = np.float32([[714, 45], [962, 46], [600, 562], [858, 575]])
out_pts = np.float32([[0, 0], [1080, 0], [0, 1350], [1080, 1350]])
M = cv.getPerspectiveTransform(int_pts, out_pts)
img_new5 = cv.warpPerspective(img1, M, (1080, 1350))


images = [img, img_new, img_new2, img_new3, img1, img_new4, img_new5]

for i in range(7):
    plt.subplot(4, 3, i+1)
    plt.imshow(images[i])
plt.show()
