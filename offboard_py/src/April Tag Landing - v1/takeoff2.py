#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.msg import ModelStates, ModelState
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from cv_bridge import CvBridge
import apriltag as ar
import cv2 as cv

class TakeOff():
    def __init__(self):
        rospy.init_node('takeoff_node', anonymous=True)
        self.velocity_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=100)
        self.tag_pub = rospy.Publisher("/artag/rgb/image_raw", Image, queue_size=100)
        self.rate = rospy.Rate(60)
        self.final_vel = TwistStamped()
        self.current_pose = NavSatFix()
        self.wamv_pose = NavSatFix()
        self.bridge = CvBridge()
        self.cv_image = []
        self.img_msg = Image()
        self.detector = ar.Detector()

        self.deltaS = 0.0
        self.targetAltitude = None
        self.linear_vel = 0
        self.yaw_vel = 0
        self.theta = 0
        self.final_vel.header.frame_id = 'map'
        self.final_vel.header.stamp = rospy.Time.now()

        rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, self.wamv)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.uav)
        rospy.Subscriber("/iris_downward_depth_camera/camera/rgb/image_raw/compressed", CompressedImage, self.artag)

        while not rospy.is_shutdown():
            self.calculate_vel()
            self.velocity_pub.publish(self.final_vel)
            self.tag_pub.publish(self.img_msg)
            rospy.loginfo("\ndrone velocity:\n{}".format(self.final_vel))
            self.rate.sleep()

    def uav(self, msg):
        self.current_pose = msg
        if self.targetAltitude is None:
            self.targetAltitude = self.current_pose.altitude + 10

    def wamv(self, msg):
        self.wamv_pose = msg

    def artag(self, msg2):
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
        R = 6378.137; #Radius of earth in KM
        lat1 = self.current_pose.latitude * np.pi/180
        lon1 = self.current_pose.longitude * np.pi/180
        lat2 = self.wamv_pose.latitude * np.pi/180
        lon2 = self.wamv_pose.longitude * np.pi/180
        dLat = lat2 - lat1
        dLon = lon2 - lon1
        ax = np.cos(lat1) * np.cos(lat2) * np.sin(dLon/2) **2
        ay = np.sin(dLat/2) **2
        cx = 2 * np.arctan2(np.sqrt(ax), np.sqrt(1-ax))
        cy = 2 * np.arctan2(np.sqrt(ay), np.sqrt(1-ay))
        deltax = R * cx * 1000 * np.sign(dLon)
        deltay = R * cy * 1000 * np.sign(dLat)

        # deltax = self.wamv_pose.pose.position.x - self.current_pose.pose.position.x
        # deltay = self.wamv_pose.pose.position.y - self.current_pose.pose.position.y
        # deltax = self.deltaS * np.cos((lat1+ lat2) / 2) * (lon2 - lon1)
        # deltay = self.deltaS * (lat2 - lat1)
        # (_, _, euler_uav_z) = euler_from_quaternion([self.current_pose.pose.orientation.x,
        #                                             self.current_pose.pose.orientation.y,
        #                                             self.current_pose.pose.orientation.z,
        #                                             self.current_pose.pose.orientation.w])
        # (_, _, euler_wamv_z) = euler_from_quaternion([self.wamv_pose.pose.orientation.x,
        #                                              self.wamv_pose.pose.orientation.y,
        #                                              self.wamv_pose.pose.orientation.z,
        #                                              self.wamv_pose.pose.orientation.w])
        # deltayaw = euler_wamv_z - euler_uav_z

        self.deltaS = np.sqrt(np.square(deltax) + np.square(deltay))
        rospy.loginfo(f"Remaining distance: {self.deltaS}")
        self.theta = np.arctan2(deltay, deltax)

        self.linear_vel = np.sqrt(2 * 0.01 * self.deltaS)
        # self.yaw_vel = np.sqrt(2 * 0.1 * np.abs(self.theta))

        self.final_vel.twist.linear.x = self.linear_vel * np.cos(self.theta)
        self.final_vel.twist.linear.y = self.linear_vel * np.sin(self.theta)
        if self.targetAltitude is not None:
            deltaz = self.targetAltitude - self.current_pose.altitude
            self.final_vel.twist.linear.z = np.sign(deltaz) * np.sqrt(2*0.05*np.abs(deltaz))

        # self.final_vel.twist.angular.z = np.sign(deltayaw) * self.yaw_vel        


if __name__ == '__main__':
    try:
        take_off = TakeOff()
    except rospy.ROSInterruptException:
        pass
