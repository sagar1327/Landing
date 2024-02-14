#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv


def callback(img_msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img_msg, 'rgb8')
    cv.imshow("Subscriber window", cv_image)
    cv.waitKey(1)
    rospy.loginfo(img_msg)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/img_pub", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
