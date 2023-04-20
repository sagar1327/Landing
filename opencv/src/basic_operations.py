#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv

img = cv.imread('f.jpg')
# cv.imread give BGR or RGB
# getting pixel intensity of a particular pixel
print(img.item(100, 100, 2))
# modifying pixel values
img.itemset((100, 100, 2), 90)

# for selecting a region (20 rows, 20 column)
px = img[100:120, 100:120]
print(px)
# accessing the 20th row of the matrix
print(px[19])
# modifying pixel values
px[0:19] = [0, 0, 0]
# cv.imshow("Display Window", img)
# cv.waitKey(0)

# image properties
print('{}, {}, and {}'.format(img.shape, img.size, img.dtype))
# copy paste
img[100:120, 100:120] = img[300:320, 300:320]
# cv.imshow("Display Window", img)
# cv.waitKey(0)

# split and merge
b, g, r = cv.split(img)
# OR
blue = img[:, :, 0]
print(blue)
r[:, :] = 0
print(r)
img = cv.merge((b, g, r))
# cv.imshow("Display Window", img)
# cv.waitKey(0)
# OR
img[:, :, 2] = 0
cv.imshow("Display Window", img)
cv.waitKey(0)
