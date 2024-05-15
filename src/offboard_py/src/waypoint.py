#!/bin/env python3

import rospy
import numpy as np
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix, CompressedImage
from cv_bridge import CvBridge
import apriltag as ar
import cv2 as cv

class Waypoints():
    def __init__(self):
        rospy.init_node("waypoint_navigation", anonymous=True)

        self.single_wp = Waypoint()
        self.single_wp.frame = 3
        self.single_wp.command = 16
        self.single_wp.autocontinue = True
        self.single_wp.is_current = False
        self.single_wp.param1 = 0.0
        self.single_wp.param2 = 0.0
        self.single_wp.param3 = 0.0
        self.single_wp.param4 = float('nan')

        self.target_wp = []
        self.target_wp_received = False
        self.target_reached = False
        self.target_reached_time = None
        self.artag_detected = False
        self.artag_center = ()
        self.state_updated = False
        self.frame_updated = False
        self.wamv_coordinate = NavSatFix
        self.wamv_coordinate_received = False
        self.uav_coordinate = NavSatFix
        self.current_state = State()
        self.rate = rospy.Rate(60)
        self.bridge = CvBridge()
        self.cv_image = []
        self.img_msg = CompressedImage()
        self.detector = ar.Detector()
        self.wp_reached = WaypointReached()

        rospy.wait_for_service("/mavros/mission/pull")
        self.pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull, persistent=True)

        rospy.wait_for_service("/mavros/mission/push")
        self.push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush, persistent=True)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, self.wamv)
        rospy.Subscriber('mavros/state', State, callback=self.monitor_state)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.uav)
        rospy.Subscriber("/iris_downward_depth_camera/camera/rgb/image_raw/compressed", CompressedImage, self.artag)
        rospy.Subscriber("/mavros/mission/reached", WaypointReached, callback=self.wp)
        self.artag_pub = rospy.Publisher("/artag/rgb/image_raw/compressed", CompressedImage, queue_size=100)

        while not rospy.is_shutdown():
            self.artag_pub.publish(self.img_msg)
            self.rate.sleep()

    def monitor_state(self, msg):
        self.current_state = msg
        self.state_updated  = True

    def wamv(self, msg):
        self.wamv_coordinate = msg
        self.wamv_coordinate_received = True

    def wp(self, msg):
        self.wp_reached = msg

    def artag(self, msg):
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
        self.artag_center = ()
        results = self.detector.detect(gray)

        if len(results) != 0:
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
                self.artag_center = (int(r.center[0]), int(r.center[1]))
                cv.circle(self.cv_image, (self.artag_center[0], self.artag_center[1]), 5, (0, 0, 255), -1)

        self.img_msg = self.bridge.cv2_to_compressed_imgmsg(self.cv_image, 'jpeg')
        self.frame_updated = True

    def push_wp(self,lat,lon,alt):
        self.single_wp.x_lat =  lat
        self.single_wp.y_long = lon
        self.single_wp.z_alt = alt

        self.target_wp.append(self.single_wp)

        # Push waypoints
        try:
            self.push(start_index=0, waypoints=self.target_wp)
            rospy.loginfo("Waypoint pushed.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        # Pull waypoints
        try:
            wp_count = self.pull().wp_received
            rospy.loginfo("Received %d waypoints. Waypoints pulled.", wp_count)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def uav(self, msg):
        self.uav_coordinate = msg

        if not self.target_wp_received and self.wamv_coordinate_received:
            rospy.loginfo(f"\nPushing wp:\n1. Lat - {self.wamv_coordinate.latitude}\n2. Lon - {self.wamv_coordinate.longitude}\n3. Alt - 6\n")
            self.push_wp(self.wamv_coordinate.latitude,self.wamv_coordinate.longitude,6)            
            self.target_wp_received = True

        if self.state_updated:
            if self.current_state.mode == "AUTO.LOITER" and not self.target_reached:
                self.target_reached_time = rospy.Time.now().to_sec()
                self.target_reached = True
                rospy.loginfo("Target wp reached.")

            if self.target_reached and (rospy.Time.now().to_sec() - self.target_reached_time) > 3 and self.frame_updated:
                if len(self.artag_center) != 0:
                    rospy.loginfo("ARTag detected.")
                elif len(self.artag_center) == 0:
                    rospy.loginfo(f"\nPushing wp:\n1. Lat - {self.wamv_coordinate.latitude}\n2. Lon - {(self.wamv_coordinate.longitude + (np.random.rand(1)*2 - 1)/20000)}\n3. Alt - 6\n")
                    self.push_wp(self.wamv_coordinate.latitude,(self.wamv_coordinate.longitude + (np.random.rand(1)*2 - 1)/10000),6)
                    mode = self.set_mode(custom_mode='AUTO.MISSION')
                    if mode.mode_sent:
                        rospy.loginfo("Moving to the next wp.")
                    self.target_reached = False

            self.state_updated = False
            self.wamv_coordinate_received = False
            self.frame_updated = False


if __name__ == '__main__':
    try:
        Waypoints()
    except rospy.ROSInterruptException:
        pass
