#!/home/sagar/ROS/opencv/env/bin/python

import cv2 as cv
import numpy as np
import time

# params for ShiTomasi corner detection
feature_params = dict(maxCorners=100,
                      qualityLevel=0.3,
                      minDistance=2,
                      blockSize=2)
# Parameters for lucas kanade optical flow
lk_params = dict(winSize=(15, 15),
                 maxLevel=15,
                 criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 30, 0.1))
# Create some random colors
color = np.random.randint(0, 255, (100, 3))

cap = cv.VideoCapture("shadows.mp4")
if not cap.isOpened():
    print("Cannot open camera")
    exit()
# Capture old frame
ret1, old_frame = cap.read()
if not ret1:
    print("Can't receive frame (stream end?). Exiting ...")
    exit()
# Resize
old_frame_resized = cv.resize(old_frame, None, fx=0.2, fy=0.2, interpolation=cv.INTER_AREA)
# Grayscale
old_gray = cv.cvtColor(old_frame_resized, cv.COLOR_BGR2GRAY)
# Thresholding
ret2, old_frame_thresh = cv.threshold(old_gray, 40, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
# Corner detection
p0 = cv.goodFeaturesToTrack(old_frame_thresh, mask=None, **feature_params)
ti = time.time()
# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame_thresh)

while True:
    # Capture new frame
    ret3, new_frame = cap.read()
    # if frame is read correctly ret is True
    if not ret3:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Resize
    new_frame_resized = cv.resize(new_frame, None, fx=0.2, fy=0.2, interpolation=cv.INTER_AREA)
    # Grayscale
    new_gray = cv.cvtColor(new_frame_resized, cv.COLOR_BGR2GRAY)
    # Thresholding
    ret4, new_frame_thresh = cv.threshold(new_gray, 40, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    # calculate optical flow
    p1, st, err = cv.calcOpticalFlowPyrLK(old_frame_thresh, new_frame_thresh, p0, None, **lk_params)
    tf = time.time()

    # Select good points
    if p1 is not None:
        good_new = p1[st == 1]
        good_old = p0[st == 1]
    else:
        break

    # draw the tracks
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
        new_frame_thresh = cv.circle(new_frame_thresh, (int(a), int(b)), 5, color[i].tolist(), -1)
    img = cv.add(new_frame_thresh, mask)
    # Display the resulting frame
    cv.imshow('frame', img)
    if cv.waitKey(1) == ord('q'):
        break

    # Distance
    dx = p1[:, 0][:, 0] - p0[:, 0][:, 0]
    dy = p1[:, 0][:, 1] - p0[:, 0][:, 1]
    d = np.sqrt(dx**2 + dy**2)
    # Speed
    v = d / (tf - ti)
    print(v)
    # Now update the previous frame and previous points
    old_gray = new_gray.copy()
    p0 = good_new.reshape(-1, 1, 2)

cv.destroyAllWindows()
