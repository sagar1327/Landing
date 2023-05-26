#!/usr/bin/env python

import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def callback(img):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, 'rgb8')
    cv.imshow("Subscriber window", cv_image)
    cv.waitKey(1)
    rospy.loginfo("Publishing....")


def listener():
    rospy.init_node("voxl camera subscriber", anonymous=True)
    rospy.Subscriber("tracking", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
