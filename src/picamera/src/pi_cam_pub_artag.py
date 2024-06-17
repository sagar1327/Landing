#!/usr/bin/env python3

import rospy
import cv2 as cv
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import apriltag as ar
import cv2 as cv

class Talker():
    def __init__(self):
        rospy.init_node('rpi_img_pub', anonymous=True)
        self.pub = rospy.Publisher('rpi/rgb/image_raw/compressed', CompressedImage, queue_size=100)
        self.rate = rospy.Rate(60)  # 60hz
        self.bridge = CvBridge()
        self.detector = ar.Detector()

        # Open the camera device
        self.cap = cv.VideoCapture("/dev/video0", cv.CAP_V4L)
        if not self.cap.isOpened():
            rospy.logerr("Error opening the camera.")
            return

        while not rospy.is_shutdown():
            self.ret, self.cv_image = self.cap.read()
            if not self.ret:
                rospy.logerr("Error reading frame from the camera.")
                break

            gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
            results = self.detector.detect(gray)

            for r in results:
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                cv.line(self.cv_image, ptA, ptB, (0, 255, 0), 2)
                cv.line(self.cv_image, ptB, ptC, (0, 255, 0), 2)
                cv.line(self.cv_image, ptC, ptD, (0, 255, 0), 2)
                cv.line(self.cv_image, ptD, ptA, (0, 255, 0), 2)
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)
                       
            # Convert the frame to a compressed image message
            self.img_msg = self.bridge.cv2_to_compressed_imgmsg(self.cv_image, dst_format='jpeg')  # Change compression format if needed

            # Publish the compressed image
            self.pub.publish(self.img_msg)
            rospy.loginfo_once("Now publishing...")
            self.rate.sleep()

        # Release the camera when the node is interrupted
        self.cap.release()

if __name__ == '__main__':
    try:
        Talker()
    except rospy.ROSInterruptException:
        pass
