#!/usr/bin/env python3

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def talker():
    pub = rospy.Publisher('img_pub', Image, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(60)  # 60hz
    bridge = CvBridge()
    cap = cv.VideoCapture("/dev/video0",cv.CAP_V4L)

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
