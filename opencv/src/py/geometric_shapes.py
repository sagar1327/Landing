#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv
import numpy as np
#
# In all the above functions, you will see some common arguments as given below:
#
# img : The image where you want to draw the shapes
# color : Color of the shape. for BGR, pass it as a tuple, eg: (255,0,0) for blue.
# For grayscale, just pass the scalar value.
# thickness : Thickness of the line or circle etc. If -1 is passed for closed figures like circles,
# it will fill the shape. default thickness = 1
# lineType : Type of line, whether 8-connected, anti-aliased line etc. By default, it is 8-connected.
# cv.LINE_AA gives anti-aliased line which looks great for curves.

# creating a white background
img = 255*np.ones((480, 640, 3), np.uint8)

# To draw a line, you need to pass starting and ending coordinates of line. We will create a black image
# and draw a blue line on it from top-left to bottom-right corners.
cv.line(img, (0, 0), (640, 480), (0, 255, 255), 5)

# To draw a rectangle, you need top-left corner and bottom-right corner of rectangle. This time we will draw a green
# rectangle in the top-right corner of image.
cv.rectangle(img, (20, 40), (620, 440), (0, 0, 255), 10)

# To draw a circle, you need its center coordinates and radius. We will draw a circle inside the rectangle drawn above.
cv.circle(img, (320, 240), 150, (30, 200, 140), -5)

# To draw a polygon, first you need coordinates of vertices.
pts = np.array([[320, 40], [20, 240], [320, 440], [620, 240]], np.int32)
cv.polylines(img, [pts], True, (255, 0, 255), 3)
# If third argument is False, you will get a polylines joining all the points, not a closed shape.

# To put texts in images, you need specify following things.
#
# Text data that you want to write
# Position coordinates of where you want put it (i.e. bottom-left corner where data starts).
# Font type (Check cv.putText() docs for supported fonts)
# Font Scale (specifies the size of font)
# regular things like color, thickness, lineType etc. For better look, lineType = cv.LINE_AA is recommended.
font = cv.FONT_HERSHEY_SIMPLEX
cv.putText(img, 'OpenCV', (90, 430), font, 4, (0, 0, 0), 2, cv.LINE_AA)

cv.imshow('Shape', img)
cv.waitKey(0)
