#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


def takeoff():
    rospy.init_node('takeoff_node', anonymous=True)
    takeoff_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    rate = rospy.Rate(20)
    pose = PoseStamped()

    while not rospy.is_shutdown():
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2
        takeoff_pub.publish(pose)
        rospy.loginfo(pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException:
        pass
