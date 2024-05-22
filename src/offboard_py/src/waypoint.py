#!/bin/env python3

import rospy
import numpy as np
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix, CompressedImage
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from cv_bridge import CvBridge
import apriltag as ar
import cv2 as cv

class Waypoints():
    def __init__(self):
        rospy.init_node("waypoint_navigation", anonymous=True)

        # Waypoint
        self.wp_reached = WaypointReached()
        self.single_wp = Waypoint()
        self.target_wp = []
        self.single_wp.frame = 3
        self.single_wp.command = 16
        self.single_wp.autocontinue = True
        self.single_wp.is_current = False
        self.single_wp.param1 = 0.0
        self.single_wp.param2 = 0.0
        self.single_wp.param3 = 0.0
        self.single_wp.param4 = float('nan')        
        self.target_wp_received = False
        self.targetWP_reached = False
        self.targetWP_reached_time = None
        # AprilTag
        options = ar.DetectorOptions(families=['tag36h11','tag25h9'],
                                 border=1,
                                 nthreads=4,
                                 quad_decimate=1.0,
                                 quad_blur=0.0,
                                 refine_edges=True,
                                 refine_decode=False,
                                 refine_pose=False,
                                 debug=False,
                                 quad_contours=True)
        self.detector = ar.Detector(options=options)
        self.artag_detected = False
        self.artag_previously_detected = False
        self.artag_center = []
        self.artag_family = []
        self.artag_detected_time = None
        # UAV state
        self.current_state = State()
        self.state_updated = False
        # UAV and WAMV coordinate
        self.wamv_coordinate = NavSatFix
        self.wamv_coordinate_received = False
        self.uav_coordinate = NavSatFix
        # UAV pose and velocity
        self.current_pose = PoseStamped()
        self.set_uav_velocity = TwistStamped()
        self.set_uav_velocity.header.frame_id = 'map'
        self.angle = (0, 0, 0)
        # Image
        self.img_msg = CompressedImage()
        self.bridge = CvBridge()
        self.frame_updated = False
        self.cv_image = []
        self.fov = 63
        self.image_size = [640, 480]
        # Landing parameters
        self.hover_alt = None
        self.hovering_time = None

        self.rate = rospy.Rate(60)

        # ROS services
        rospy.wait_for_service("/mavros/mission/pull")
        self.pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull, persistent=True)
        rospy.wait_for_service("/mavros/mission/push")
        self.push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush, persistent=True)
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # ROS Subscribers
        rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, self.wamv)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.uav)
        rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback=self.uav_pose)
        rospy.Subscriber('mavros/state', State, callback=self.monitor_state)
        rospy.Subscriber("/iris_downward_depth_camera/camera/rgb/image_raw/compressed", CompressedImage, self.artag)
        rospy.Subscriber("/mavros/mission/reached", WaypointReached, callback=self.wp)
        rospy.Subscriber('mavros/rc/in', RCIn, callback=self.rc_callback)

        # ROS Publishers
        self.artag_pub = rospy.Publisher("/artag/rgb/image_raw/compressed", CompressedImage, queue_size=100)
        self.velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=100)

        while not rospy.is_shutdown():
            self.velocity_pub.publish(self.set_uav_velocity)
            self.rate.sleep()

        # rospy.spin()

    def monitor_state(self, msg):
        self.current_state = msg
        self.state_updated  = True

    def wamv(self, msg):
        self.wamv_coordinate = msg
        self.wamv_coordinate_received = True

    def wp(self, msg):
        self.wp_reached = msg

    def uav_pose(self, msg):
        self.current_pose = msg
        self.angle = euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        # rospy.loginfo(f"\nHeading: {self.angle[2]}\n")

    def rc_callback(self,msg):
        # Check if the RC switch is in the "Position control" position (Means manual control)
        if msg.channels[4] < 1300:
            # Arm the UAV
            rospy.loginfo_once("Vehicle is in position mode (RC switch). Shutting down script")
            self.manual_control = True
            rospy.signal_shutdown("Manual control activated")

    def artag(self, msg, hover=False):
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
        self.artag_center = []
        self.artag_family = []
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
                self.artag_center.append([int(r.center[0]), int(r.center[1])])
                self.artag_family.append(str(r.tag_family))
                cv.circle(self.cv_image, (int(r.center[0]), int(r.center[1])), 5, (0, 0, 255), -1)

            # rospy.loginfo(self.artag_family)
        self.img_msg = self.bridge.cv2_to_compressed_imgmsg(self.cv_image, 'jpeg')
        self.artag_pub.publish(self.img_msg)
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

    def align(self, hover=False, tagfamily="b'tag36h11'"):
        apx = self.fov/self.image_size[0]
        apy = self.fov/self.image_size[1]
        # Image frame
        desired_center = self.artag_center[self.artag_family.index(tagfamily)]
        delta_pixel_x = desired_center[0] - self.image_size[0]/2
        delta_pixel_y = desired_center[1] - self.image_size[1]/2
        alpha = np.abs(delta_pixel_x)*apx*np.pi/180
        beta = np.abs(delta_pixel_y)*apy*np.pi/180
        deltax_img = np.sign(delta_pixel_x)*np.tan(alpha)*self.current_pose.pose.position.z
        deltay_img = -np.sign(delta_pixel_y)*np.tan(beta)*self.current_pose.pose.position.z
        deltaS_img = np.sqrt(np.square(deltax_img) + np.square(deltay_img))
        theta_img = np.arctan2(deltay_img, deltax_img)

        # #Global frame
        deltaS = deltaS_img
        # theta = theta_img+self.angle[2]-np.pi/2

        # linear_vel = np.sqrt(2 * 0.01 * deltaS)
        # self.set_uav_velocity.header.frame_id = 'map'
        # self.set_uav_velocity.header.stamp = rospy.Time.now()
        # self.set_uav_velocity.twist.linear.x = linear_vel * np.cos(theta)
        # self.set_uav_velocity.twist.linear.y = linear_vel * np.sin(theta)
        # self.set_uav_velocity.twist.linear.z = 0.0

        # rospy.loginfo(f"\napx: {apx}\napy: {apy}\nARTag center: {self.artag_center}\ndelta_pixel_x: {delta_pixel_x}"+
        #               f"\ndelta_pixel_y: {delta_pixel_y}\nalpha: {alpha}\nbeta: {beta}\nCurrent height: {self.current_pose.pose.position.z}\ndeltax_img: {deltax_img}\ndeltay_img: {deltay_img}"+
        #               f"\ndeltaS_img: {deltaS_img}\ntheta_img: {theta_img}\nHeading: {self.angle[2]}\ntheta: {theta}"+
        #               f"\nlinear_vel: {linear_vel}\nlinearx_vel: {self.set_uav_velocity.twist.linear.x}\nlineary_vel: {self.set_uav_velocity.twist.linear.y}\n")

        # Calculate the angle between the estimated position and the target position
        theta_horizontal = theta_img+self.angle[2]-np.pi/2 #Angle to the target in x-y plane

        theta_vertical = np.arctan2(deltaS, self.current_pose.pose.position.z) #Angle to the target in relative to straight down plane

        # Calculate the linear velocity based on the distance
        gain = 0.3
        self.linear_vel = gain * deltaS
        self.linear_vel = 0.8 if self.linear_vel > 0.8 else self.linear_vel #Avoid going too fast to be safe was 0.5
        ##################################
        desired_heading = np.pi/2
        current_heading = self.angle[2]
        gain_heading = 0.2
        angular_z_vel = gain_heading*(desired_heading-current_heading)
        self.set_uav_velocity.twist.angular.z = angular_z_vel

        # Publish the correction position if the distance is greater than 0.2
        if hover:

            if deltaS <= 0.2:
                mode = self.set_mode(custom_mode='AUTO.LAND')
                if mode.mode_sent:
                    rospy.loginfo("Landing.")
                rospy.signal_shutdown("Closing script.")

            rospy.loginfo("Aligining to land.")
            #P controller to maintain altitude
            desired_alt =  0.3#Hover at 1.3 m
            delta_alt = desired_alt - self.current_pose.pose.position.z
            gain_alt = 0.8
            self.set_uav_velocity.twist.linear.z = gain_alt*delta_alt
            
            # Set the linear velocity components in the x and y directions
            self.set_uav_velocity.header.stamp = rospy.Time.now()
            self.set_uav_velocity.twist.linear.x = self.linear_vel * np.cos(theta_horizontal)
            self.set_uav_velocity.twist.linear.y = self.linear_vel * np.sin(theta_horizontal)

            if self.hovering_time is None:
                self.hovering_time = rospy.Time.now().to_sec()
            
        elif theta_vertical > 8*np.pi/180: #Not well aligned in z (more than 10 deg off) or in final aligning stage
            # rospy.loginfo("Aligning")
            
            # # Set the linear velocity components in the x and y directions
            # self.set_uav_velocity.header.stamp = rospy.Time.now()
            # self.set_uav_velocity.twist.linear.x = self.linear_vel * np.cos(theta_horizontal)
            # self.set_uav_velocity.twist.linear.y = self.linear_vel * np.sin(theta_horizontal)
            # self.set_uav_velocity.twist.linear.z = 0

            rospy.loginfo("Hovering and aligning")
            #P controller to maintain altitude
            if self.hover_alt is None:
                self.hover_alt =  self.current_pose.pose.position.z#Hover at 1.3 m
            delta_alt = self.hover_alt - self.current_pose.pose.position.z
            gain_alt = 0.8
            self.set_uav_velocity.twist.linear.z = gain_alt*delta_alt
            
            # Set the linear velocity components in the x and y directions
            self.set_uav_velocity.header.stamp = rospy.Time.now()
            self.set_uav_velocity.twist.linear.x = self.linear_vel * np.cos(theta_horizontal)
            self.set_uav_velocity.twist.linear.y = self.linear_vel * np.sin(theta_horizontal)

        else:
            rospy.loginfo("Descending")
            # Set the linear velocity components in the x and y directions
            self.set_uav_velocity.header.stamp = rospy.Time.now()
            self.set_uav_velocity.twist.linear.x = self.linear_vel * np.cos(theta_horizontal)
            self.set_uav_velocity.twist.linear.y = self.linear_vel * np.sin(theta_horizontal)
            self.set_uav_velocity.twist.linear.z = -0.5 #Descend with 0.2 m/s

            if self.hover_alt is not None:
                self.hover_alt = None

    def uav(self, msg):
        self.uav_coordinate = msg

        if not self.target_wp_received and self.wamv_coordinate_received:
            rospy.loginfo(f"\nPushing wp:\n1. Lat - {self.wamv_coordinate.latitude}\n2. Lon - {self.wamv_coordinate.longitude - 0.00005}\n3. Alt - 6\n")
            self.push_wp(self.wamv_coordinate.latitude,self.wamv_coordinate.longitude - 0.00005,6)            
            self.target_wp_received = True

        if self.state_updated:
            if self.current_state.mode == "AUTO.LOITER" and not self.targetWP_reached:
                self.targetWP_reached_time = rospy.Time.now().to_sec()
                self.targetWP_reached = True
                rospy.loginfo("Target wp reached.")

            if self.targetWP_reached and self.frame_updated:
                
                if len(self.artag_center) != 0:
                    self.align(hover=True if self.current_pose.pose.position.z <=0.3 else False,
                               tagfamily="b'tag25h9'" if "b'tag25h9'" in self.artag_family else "b'tag36h11'")

                    if self.artag_detected_time is None:
                        self.artag_detected_time = rospy.Time.now().to_sec()
                        
                    if (rospy.Time.now().to_sec() - self.artag_detected_time) > 3 and self.current_state.mode == "AUTO.LOITER":
                        mode = self.set_mode(custom_mode='OFFBOARD')
                        if mode.mode_sent:
                            rospy.loginfo("Moving towards the ARTag.")
                        self.artag_previously_detected = True
                        
                elif len(self.artag_center) == 0:
                    if self.artag_previously_detected:
                        self.set_uav_velocity.header.stamp = rospy.Time.now()
                        self.set_uav_velocity.twist.linear.x = 0
                        self.set_uav_velocity.twist.linear.y = 0
                        self.set_uav_velocity.twist.linear.z = 0.3
                    else:
                        rospy.loginfo(f"\nPushing wp:\n1. Lat - {self.wamv_coordinate.latitude}\n2. Lon - {self.wamv_coordinate.longitude + (np.random.rand(1)*2 - 1)/30000}\n3. Alt - 6\n")
                        self.push_wp(self.wamv_coordinate.latitude,(self.wamv_coordinate.longitude + (np.random.rand(1)- 0.5)/10000),6)
                        mode = self.set_mode(custom_mode='AUTO.MISSION')
                        if mode.mode_sent:
                            rospy.loginfo("Moving to the next wp.")
                        self.targetWP_reached = False

            self.state_updated = False
            self.wamv_coordinate_received = False
            self.frame_updated = False


if __name__ == '__main__':
    try:
        Waypoints()
    except rospy.ROSInterruptException:
        pass
