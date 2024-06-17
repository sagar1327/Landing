#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 as cv

class Listener():
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = CompressedImage()

        # Subscribing to the compressed image message
        rospy.init_node('rpi_img_sub', anonymous=True)
        rospy.Subscriber("/rpi/rgb/image_raw/compressed", CompressedImage, self.callback)
        rospy.spin()

    # Define custom callback.
    def callback(self,img_msg):
        # Convert the ros compressed image to a opencv image
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, 'bgr8')
        rospy.loginfo_once("Now Subscribing...")


if __name__ == '__main__':
    try:
        Listener()
    except rospy.ROSInterruptException:
        # Destroy the all windows when node is interrupted
        cv.destroyAllWindows
        pass
