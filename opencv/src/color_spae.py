#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv
import numpy as np

# HSV, Hue range 0-179, Saturation 0-255, Value 0-255

# img = cv.imread("test_img2.jpg")
# cv.imshow("Display", img)
# cv.waitKey(0)
# cv.destroyAllWindows()
# # To get the HSV value from BGR values
# true_color = np.uint8([[[202, 126, 67]]])
# img_hsv = cv.cvtColor(true_color, cv.COLOR_BGR2HSV)
# print(img_hsv)

cap = cv.VideoCapture(0)
while 'true':
    _, frame = cap.read()
    img_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower_limit = np.array([100, 50, 50])  # blue marker
    upper_limit = np.array([110, 255, 255])

    mask = cv.inRange(img_hsv, lower_limit, upper_limit)
    final = cv.bitwise_and(frame, frame, mask=mask)

    cv.imshow("Display", final)
    if cv.waitKey(1) == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
