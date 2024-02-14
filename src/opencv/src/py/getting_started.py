#!/usr/bin/env python

import cv2 as cv
import sys
img = cv.imread(cv.samples.findFile("f.jpg"))
if img is None:
    sys.exit("Could not read the image.")
cv.imshow("Display window", img)
k = cv.waitKey(0)
# cvWaitKey(x) stops your program until you press a button.
# x<=0, it waits indefinitely for the key press.
if k == ord("s"):
    cv.imwrite("flower.png", img)
    
