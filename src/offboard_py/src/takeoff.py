#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.msg import ModelStates, ModelState
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import apriltag as ar
import cv2 as cv

class TakeOff():
    def __init__(self):
        rospy.init_node('takeoff_node', anonymous=True)
        self.velocity_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=100)
        self.tag_pub = rospy.Publisher("/artag/rgb/image_raw", Image, queue_size=100)
        self.rate = rospy.Rate(60)
        self.current_vel = TwistStamped()
        self.final_vel = TwistStamped()
        self.current_pose = ModelState()
        self.wamv_pose = ModelState()
        self.bridge = CvBridge()
        self.cv_image = []
        self.img_msg = Image()
        self.detector = ar.Detector()

        self.deltaS = 0.0
        self.linear_vel = 0
        self.yaw_vel = 0
        self.theta = 0
        self.final_vel.header.frame_id = 'map'
        self.final_vel.header.stamp = rospy.Time.now()

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback1)
        rospy.Subscriber("/iris_downward_depth_camera/camera/rgb/image_raw/compressed", CompressedImage, self.callback2)

        while not rospy.is_shutdown():
            self.calculate_vel()
            self.velocity_pub.publish(self.final_vel)
            self.tag_pub.publish(self.img_msg)
            rospy.loginfo("\ndrone velocity:\n{}".format(self.final_vel))
            self.rate.sleep()

    def callback1(self, msg1):
        if msg1.name[22] == "wamv":
            self.current_pose.pose = msg1.pose[23]
            self.wamv_pose.pose = msg1.pose[22]
        else:
            self.current_pose.pose = msg1.pose[22]
            self.wamv_pose.pose = msg1.pose[23]

    def callback2(self, msg2):
        # rospy.loginfo("{}x{}\n".format(msg.height, msg.width))
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg2, 'bgr8')
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

        self.img_msg = self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8')

    def calculate_vel(self):
        deltax = self.wamv_pose.pose.position.x - self.current_pose.pose.position.x
        deltay = self.wamv_pose.pose.position.y - self.current_pose.pose.position.y
        deltaz = 4 - self.current_pose.pose.position.z
        (_, _, euler_uav_z) = euler_from_quaternion([self.current_pose.pose.orientation.x,
                                                    self.current_pose.pose.orientation.y,
                                                    self.current_pose.pose.orientation.z,
                                                    self.current_pose.pose.orientation.w])
        (_, _, euler_wamv_z) = euler_from_quaternion([self.wamv_pose.pose.orientation.x,
                                                     self.wamv_pose.pose.orientation.y,
                                                     self.wamv_pose.pose.orientation.z,
                                                     self.wamv_pose.pose.orientation.w])
        deltayaw = euler_wamv_z - euler_uav_z

        self.deltaS = np.sqrt(np.square(deltax) + np.square(deltay))
        rospy.loginfo(f"Remaining distance: {self.deltaS}")
        self.theta = np.arctan2(deltay, deltax)

        self.linear_vel = np.sqrt(2 * 1 * self.deltaS)
        self.yaw_vel = np.sqrt(2 * 0.2 * np.abs(deltayaw))

        self.final_vel.twist.linear.x = self.linear_vel * np.cos(self.theta)
        self.final_vel.twist.linear.y = self.linear_vel * np.sin(self.theta)
        self.final_vel.twist.linear.z = np.sign(deltaz) * np.sqrt(2*2*np.abs(deltaz))
        self.final_vel.twist.angular.z = np.sign(deltayaw) * self.yaw_vel        


if __name__ == '__main__':
    try:
        take_off = TakeOff()
    except rospy.ROSInterruptException:
        pass
