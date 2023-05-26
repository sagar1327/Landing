#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def talker():
    pub = rospy.Publisher('chatter', Image, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(60)  # 10hz
    lst = Image()
    bridge = CvBridge()
    cap = cv.VideoCapture(0)
 
    while not rospy.is_shutdown():
       ret, frame = cap.read()
       img_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
       pub.publish(img_msg)
       rospy.loginfo("Now Publishing.....")
       rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
