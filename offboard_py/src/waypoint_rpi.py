#!/bin/env python3

import rospy
import numpy as np
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix, CompressedImage
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge
import apriltag as ar
import cv2 as cv

class Waypoints():
    def __init__(self):
        
        # Waypoint.
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
        # AprilTag.
        options = ar.DetectorOptions(families=['tag36h11','tag25h9'],
                                    border=1,
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_blur=0.0,
                                    refine_edges=True,
                                    refine_decode=False,
                                    refine_pose=False,
                                    debug=False,
                                    quad_contours=False)
        self.detector = ar.Detector(options=options)
        self.artag_detected = False
        self.artag_previously_detected = False
        self.artag_center = []
        self.artag_family = []
        self.artag_detected_time = None
        self.mainARTag_detected_time = None
        self.deltaS = np.Inf
        self.artag_lost_time = None
        # UAV state.
        self.current_state = State()
        self.state_updated = False
        self.mode_changed_to_mission = False
        # UAV pose and velocity.
        self.current_pose = PoseStamped()
        self.current_alt = 0.0
        self.actual_alt = 0.0
        self.set_uav_velocity = TwistStamped()
        self.set_uav_velocity.header.frame_id = 'map'
        self.angle = (0, 0, 0)
        self.pose_updated = False
        # Image.
        self.img_msg = CompressedImage()
        self.bridge = CvBridge()
        self.frame_updated = False
        self.cv_image = []
        self.fov = 63
        self.image_size = [640, 480]
        # Landing parameters.
        self.hover_alt = None
        self.hovering_time = None
        self.land_time = None
        self.proximity = []
        self.landing_condition = False

    def monitor_state(self, msg):
        self.current_state = msg
        self.state_updated  = True

    def wp(self, msg):
        self.wp_reached = msg

    def uav_pose(self, msg):
        self.current_pose = msg
        if self.state_updated and not self.current_state.armed:
            self.current_alt = self.current_pose.pose.position.z
        self.actual_alt = self.current_pose.pose.position.z - self.current_alt
        rospy.loginfo_throttle(2,f"\nCurrent Alt: {self.actual_alt}")
        self.angle = euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        self.pose_updated = True

    def rc_callback(self,msg):
        # Check if the RC switch is in the "Position control" position (Means manual control).
        if msg.channels[4] < 1300:
            rospy.loginfo_once("\nVehicle is in position mode (RC switch). Shutting down script\n")
            self.manual_control = True
            rospy.signal_shutdown("\nManual control activated\n")

    def artag(self, msg):
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        self.cv_image = cv.rotate(self.cv_image, cv.ROTATE_180)
        gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
        self.artag_center = []
        self.artag_family = []
        # rospy.loginfo("Detected")
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
                self.artag_family.append(str(r.tag_family.decode("utf-8")))
                cv.circle(self.cv_image, (int(r.center[0]), int(r.center[1])), 2, (0, 0, 255), -1)

        self.img_msg = self.bridge.cv2_to_compressed_imgmsg(self.cv_image, 'jpeg')
        self.frame_updated = True

    def push_wp(self,push,pull,lat,lon,alt):
        self.single_wp.x_lat =  lat
        self.single_wp.y_long = lon
        self.single_wp.z_alt = alt

        self.target_wp.append(self.single_wp)

        # Push waypoints.
        try:
            push(start_index=0, waypoints=self.target_wp)
            rospy.loginfo("\nWaypoint pushed\n.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        # Pull waypoints.
        try:
            wp_count = pull().wp_received
            rospy.loginfo("\nReceived waypoint %d. Waypoints pulled.\n", wp_count)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def align(self, tagfamily="tag36h11"):
        apx = self.fov/self.image_size[0]
        apy = self.fov/self.image_size[1]
        # Image frame.
        desired_center = self.artag_center[self.artag_family.index(tagfamily)]
        delta_pixel_x = desired_center[0] - self.image_size[0]/2
        delta_pixel_y = desired_center[1] - self.image_size[1]/2
        alpha = np.abs(delta_pixel_x)*apx*np.pi/180
        beta = np.abs(delta_pixel_y)*apy*np.pi/180
        deltax_img = np.sign(delta_pixel_x)*np.tan(alpha)*self.actual_alt
        deltay_img = -np.sign(delta_pixel_y)*np.tan(beta)*self.actual_alt
        deltaS_img = np.sqrt(np.square(deltax_img) + np.square(deltay_img))
        theta_img = np.arctan2(deltay_img, deltax_img)

        # #Global frame.
        self.deltaS = deltaS_img
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
        theta_vertical = np.arctan2(self.deltaS, self.actual_alt) #Angle to the target in relative to straight down plane

        # Calculate the linear velocity based on the distance.
        if self.actual_alt > 0.5:
            gain = 0.3
        else:
            gain = 1
        self.linear_vel = gain * self.deltaS
        # self.linear_vel = 0.8 if self.linear_vel > 0.8 else self.linear_vel #Avoid going too fast to be safe was 0.5.
        ##################################
        desired_heading = np.pi/2
        current_heading = self.angle[2]
        gain_heading = 0.2
        angular_z_vel = gain_heading*(desired_heading-current_heading)
        self.set_uav_velocity.twist.angular.z = angular_z_vel

        # Publish the correction position if the distance is greater than 0.2.
        if theta_vertical > 8*np.pi/180:
            if self.current_state.mode == "OFFBOARD":
                rospy.loginfo("\nHovering and aligning\n")
            #P controller to maintain altitude.
            if self.hover_alt is None:
                self.hover_alt =  self.actual_alt
            delta_alt = self.hover_alt - self.actual_alt
            gain_alt = 0.8
            self.set_uav_velocity.twist.linear.z = gain_alt*delta_alt
            
            # Set the linear velocity components in the x and y directions.
            self.set_uav_velocity.header.stamp = rospy.Time.now()
            self.set_uav_velocity.twist.linear.x = self.linear_vel * np.cos(theta_horizontal)
            self.set_uav_velocity.twist.linear.y = self.linear_vel * np.sin(theta_horizontal)

        else:
            if self.current_state.mode == "OFFBOARD":
                rospy.loginfo("\nDescending\n")
            # Set the linear velocity components in the x and y directions.
            self.set_uav_velocity.header.stamp = rospy.Time.now()
            self.set_uav_velocity.twist.linear.x = self.linear_vel * np.cos(theta_horizontal)
            self.set_uav_velocity.twist.linear.y = self.linear_vel * np.sin(theta_horizontal)
            self.set_uav_velocity.twist.linear.z = -0.5 #Descend with 0.2 m/s.

            if self.hover_alt is not None:
                self.hover_alt = None

    def land(self):
        if self.land_time is None:
            self.land_time = rospy.Time.now().to_sec()
        self.proximity.append(self.deltaS)
        # rospy.loginfo(f"\nArray: {self.proximity}\n")
        if (rospy.Time.now().to_sec() - self.land_time) > 1:
            # rospy.loginfo(f"\nProximity: {np.mean(self.proximity)}\nActual Alt: {self.actual_alt}")
            if np.mean(self.proximity) < 0.1 and self.actual_alt < 0.5:
                self.landing_condition = True 
            else:
                self.land_time = None
                self.proximity = []
            


def main():
    WP = Waypoints()
    rospy.init_node("waypoint_navigation", anonymous=True)

    # ROS services.
    rospy.wait_for_service("/mavros/mission/pull")
    pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull, persistent=True)
    rospy.wait_for_service("/mavros/mission/push")
    push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush, persistent=True)
    rospy.wait_for_service('/mavros/set_mode')
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    # ROS Subscribers.
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback=WP.uav_pose)
    rospy.Subscriber('mavros/state', State, callback=WP.monitor_state)
    rospy.Subscriber("/rpi/rgb/image_raw/compressed", CompressedImage, WP.artag)
    rospy.Subscriber("/mavros/mission/reached", WaypointReached, callback=WP.wp)
    rospy.Subscriber('mavros/rc/in', RCIn, callback=WP.rc_callback)

    # ROS Publishers.
    artag_pub = rospy.Publisher("/artag/rgb/image_raw/compressed", CompressedImage, queue_size=1)
    velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=3)

    wy = [29.1826640,-81.044177,3]

    rate = rospy.Rate(60)

    
    while not rospy.is_shutdown():

        # Push and Pull initial waypoint of the boat.
        if not WP.target_wp_received:
            rospy.loginfo(f"\nPushing wp:\n1. Lat - {wy[0]}\n2. Lon - {wy[1]}\n3. Alt - {wy[2]}\n")
            WP.push_wp(push,pull,wy[0],wy[1],wy[2])            
            WP.target_wp_received = True

        # Switch to mission mode
        if WP.current_state.armed and not WP.mode_changed_to_mission:
            mode = set_mode(custom_mode='AUTO.MISSION')
            if mode.mode_sent:
                rospy.loginfo("\nMode changed to mission. Executing the current mission.\n")
            WP.mode_changed_to_mission = True

        # If uav reach the target waypoint.
        if WP.state_updated and WP.current_state.mode == "AUTO.LOITER" and not WP.targetWP_reached:
            WP.targetWP_reached_time = rospy.Time.now().to_sec()
            WP.targetWP_reached = True
            rospy.loginfo("\nTarget wp reached\n.")

        # If a artag is detected at the target waypoint.
        if WP.targetWP_reached and (rospy.Time.now().to_sec() - WP.targetWP_reached_time) > 5 and WP.pose_updated and WP.frame_updated and len(WP.artag_center) != 0:
            
            rospy.loginfo("Detected.")

            # Check Landing condition.
            WP.land()
            if WP.landing_condition:               
                mode = set_mode(custom_mode='AUTO.LAND')
                if mode.mode_sent:
                    rospy.loginfo("Landing.")
                break
            
            # Initiate a timer.
            if WP.artag_detected_time is None:
                WP.artag_detected_time = rospy.Time.now().to_sec()

            # Start publishing required velocity to move towards the target.
            if "tag25h9" in WP.artag_family and WP.mainARTag_detected_time is None:
                WP.mainARTag_detected_time = rospy.Time.now().to_sec()

            if WP.current_state.mode != "OFFBOARD":
                if "tag36h11" not in WP.artag_family or ("tag25h9" in WP.artag_family \
                                and (rospy.Time.now().to_sec() - WP.mainARTag_detected_time) > 1):
                    WP.align(tagfamily="tag25h9")
                    rospy.loginfo("Using the smaller tag.")
                else:
                    WP.align()  

                # After few seconds of timer initiation, change the flight mode. 
                if (rospy.Time.now().to_sec() - WP.artag_detected_time) > 0.5 and WP.current_state.mode == "AUTO.LOITER":
                    mode = set_mode(custom_mode='OFFBOARD')
                    WP.artag_previously_detected = True
            else:
                if (rospy.Time.now().to_sec() - WP.artag_detected_time) > 0.5:
                    if "tag36h11" not in WP.artag_family or ("tag25h9" in WP.artag_family \
                                and (rospy.Time.now().to_sec() - WP.mainARTag_detected_time) > 1):
                        WP.align(tagfamily="tag25h9")
                    else:
                        rospy.loginfo("Using the smaller tag.")
                        WP.align()

            if WP.artag_lost_time is not None:
                    WP.artag_lost_time = None 

        # If a artag is not detected at the target waypoint.        
        elif WP.targetWP_reached and (rospy.Time.now().to_sec() - WP.targetWP_reached_time) > 3 and WP.frame_updated and len(WP.artag_center) == 0:

            # Ascend if it was previously detected at the location.
            if WP.artag_previously_detected:
                if WP.artag_lost_time is None:
                    mode = set_mode(custom_mode='AUTO.LOITER')
                    if mode.mode_sent:
                        rospy.loginfo("\nTarget lost but previously detected. Loiter for 1 secs.\n")
                        WP.artag_lost_time = rospy.Time.now().to_sec()

                elif (rospy.Time.now().to_sec() - WP.artag_lost_time) > 0.5:
                    WP.set_uav_velocity.header.stamp = rospy.Time.now()
                    WP.set_uav_velocity.twist.linear.x = 0
                    WP.set_uav_velocity.twist.linear.y = 0
                    WP.set_uav_velocity.twist.linear.z = 0.3
                
                elif (rospy.Time.now().to_sec() - WP.artag_lost_time) > 1:
                    if WP.current_state.mode == 'AUTO.LOITER':
                        mode = set_mode(custom_mode='OFFBOARD')
                        if mode.mode_sent:
                            rospy.loginfo_throttle(2,"\nLost Target for more than 1 secs. Ascending.\n")
                    WP.set_uav_velocity.header.stamp = rospy.Time.now()
                    WP.set_uav_velocity.twist.linear.x = 0
                    WP.set_uav_velocity.twist.linear.y = 0
                    WP.set_uav_velocity.twist.linear.z = 0.3
                
                if WP.artag_detected_time is not None:
                    WP.artag_detected_time = None
            else:
                rospy.loginfo("No Apriltag.")
            

        # Publish img mssg only if the a new frame is received.
        if WP.frame_updated:
            artag_pub.publish(WP.img_msg)
        # Publish velocity mssg (Even empty)
        velocity_pub.publish(WP.set_uav_velocity)

        WP.state_updated = WP.frame_updated = WP.pose_updated = False
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
