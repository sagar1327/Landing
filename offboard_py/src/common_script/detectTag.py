#!/usr/bin/env python3

import apriltag as ar
import cv2 as cv


class DetetcAprilTag():
    """Test apriltag detetction on an image or a camera frame."""
    def __init__(self):
        options = ar.DetectorOptions(families=['tag36h11','tag25h9'],
                                    border=1,
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_blur=0.0,
                                    refine_edges=True,
                                    refine_decode=False,
                                    refine_pose=False,
                                    debug=False,
                                    quad_contours=True)
        self.detector = ar.Detector(options=options)

        self.gray = []

        # ## When manually inputting image path
        # self.frame = cv.imread("/home/sagar/personal_ws/src/Landing/offboard_py/src/common_script/testTag.png")
        # self.detectTag()
        # cv.imshow('Processed Image', self.gray)
        # cv.waitKey(0)
        # cv.destroyAllWindows()

        ## When using camera
        self.cap = cv.VideoCapture("/dev/video0", cv.CAP_V4L)
        if not self.cap.isOpened():
            print("Error opening the camera.")
            return

        while self.cap.isOpened():
            self.ret, self.frame = self.cap.read()
            if not self.ret:
                print("Error reading frame from the camera.")
                break

            self.detectTag()
            cv.imshow('Processed Image', self.frame)
            if cv.waitKey(1) == ord('q'):
                break

        # Release the camera when the node is interrupted
        self.cap.release()
        cv.destroyAllWindows()

    def detectTag(self):
        self.gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
        # target_idx = self.gray > 150
        # self.gray[target_idx] = 255
        # self.gray[~target_idx] = 0
        results = self.detector.detect(self.gray)

        if len(results) != 0:
            for r in results:
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))

                cv.line(self.frame, ptA, ptB, (0, 255, 0), 2)
                cv.line(self.frame, ptB, ptC, (0, 255, 0), 2)
                cv.line(self.frame, ptC, ptD, (0, 255, 0), 2)
                cv.line(self.frame, ptD, ptA, (0, 255, 0), 2)
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv.circle(self.frame, (cX, cY), 5, (0, 0, 255), -1)


if __name__ == "__main__":
    try:
        DetetcAprilTag()
    except KeyboardInterrupt:
        pass