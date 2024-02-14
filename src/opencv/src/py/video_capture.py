#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv
#url = "http://10.187.121.61:8080/video"
cap = cv.VideoCapture("/dev/video0")
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    print(type(frame))
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    #frame_resized = cv.resize(frame, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
    # Our operations on the frame come here
    #gray = cv.cvtColor(frame_resized, cv.COLOR_BGR2GRAY)
    # Display the resulting frame
    cv.imshow('Display', frame)
    if cv.waitKey(1) == ord('q'):
        break
        

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
