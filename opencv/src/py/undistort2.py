import cv2 as cv
import numpy as np

cap = cv.VideoCapture('chessboard.mp4')
data = np.load('tracking_camera_intrinsic_data.npz')
mtx = data['camera_matrix']
dist = data['distortion_coefficient']
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        exit()
    h, w = frame.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1)
    # undistort
    mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
    dst = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    cv.imshow('Display', dst)
    if cv.waitKey(0) == ord('q'):
        break
cv.destroyAllWindows()
