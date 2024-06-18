#!/usr/bin/env python3

import rospy
import cv2 as cv
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class Talker():
    def __init__(self):
        rospy.init_node('rpi_img_pub', anonymous=True)
        self.pub = rospy.Publisher('rpi/rgb/image_raw/compressed', CompressedImage, queue_size=100)
        self.rate = rospy.Rate(60)  # 60hz
        self.bridge = CvBridge()

        # Open the camera device
        self.cap = cv.VideoCapture("/dev/video0", cv.CAP_V4L)
        if not self.cap.isOpened():
            rospy.logerr("Error opening the camera.")
            return

        while not rospy.is_shutdown():
            self.ret, self.frame = self.cap.read()
            if not self.ret:
                rospy.logerr("Error reading frame from the camera.")
                break

            # Convert the frame to a compressed image message
            self.img_msg = self.bridge.cv2_to_compressed_imgmsg(self.frame, dst_format='jpeg')  # Change compression format if needed

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
