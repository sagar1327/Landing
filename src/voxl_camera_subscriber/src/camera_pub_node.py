#!/usr/bin/env python

import rospy
from PIL import Image as Im
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge


def talker():
    pub = rospy.Publisher('chatter', Image, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(60)  # 10hz
    while not rospy.is_shutdown():
        lst = Image()
        bridge = CvBridge()
        img = Im.open('/home/sagar/ros_ws/src/voxl_camera_subscriber/src/f.jpg', mode='r')
        lst.height = img.height
        lst.width = img.width
        lst.encoding = 'rgb8'
        lst.step = img.width
        lst.data = np.array(img)
        cv_image = bridge.imgmsg_to_cv2(lst, 'mono8')
        cv_image = bridge.cv2_to_imgmsg(cv_image, 'mono8')

        pub.publish(cv_image)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
