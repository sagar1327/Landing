import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

img = cv.imread('frame1.png')
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
ret, otsu_thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
otsu_mask = cv.bitwise_not(otsu_thresh)
corners = cv.goodFeaturesToTrack(otsu_mask, 90, 0.1, 10)
corners = np.int0(corners)
for i in corners:
    x, y = i.ravel()
    cv.circle(img, (x, y), 3, 255, -1)
   
print(img.shape)
plt.imshow(img), plt.show()
