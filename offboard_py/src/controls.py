#!/usr/bin/env python3

import rospy
import numpy as np
from offboard_py.msg import ArTag, ArTagAltitude, MissionStatus
from mavros_msgs.msg import State, WaypointReached
from geometry_msgs.msg import  TwistStamped, PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool


class Controls():

    def __init__(self):

        rospy.init_node("uav_controls", anonymous=True)

        self.artag_msg = ArTag()
        self.artag_alt_msg = ArTagAltitude()
        self.uav_state_msg = State()
        self.uav_vel_msg = TwistStamped()
        self.uav_vel_msg.header.frame_id = 'map'
        self.current_pose = PoseStamped()
        self.wp_reached_msg = WaypointReached()

        self.deltaS = np.Inf
        self.land_time = None
        self.proximity = []
        self.landing_condition = False

        rospy.Subscriber("/kevin/artag", ArTag, callback=self.artag)
        rospy.Subscriber("/mavros/state", State, callback=self.uav_state)
        rospy.Subscriber("/kevin/artag/altitude", ArTagAltitude, callback=self.artag_alt)
        rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback=self.uav_pose)
        rospy.Subscriber("/mavros/mission/reached", WaypointReached, callback=self.wp_reached)

        self.uav_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=3)
        self.landing_pub = rospy.Publisher("/kevin/landing_status", Bool, queue_size=1)
        self.new_mission_pub = rospy.Publisher("/kevin/mission/status", MissionStatus, queue_size=1)
        self.rate = rospy.Rate(60)

    def artag(self, msg):
        self.artag_msg = msg

    def artag_alt(self, msg):
        self.artag_alt_msg = msg

    def uav_state(self, msg):
        self.uav_state_msg = msg

    def wp_reached(self, msg):
        self.wp_reached_msg = msg

    def uav_pose(self, msg):
        self.current_pose = msg
        self.angle = euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])

    def align(self, tagfamily="tag36h11"):
        apx = 63/640
        apy = 63/480
        tag_centers = self.artag_msg.centers
        tag_families = self.artag_msg.family_names
        tag_alt = self.artag_alt_msg.altitude

        # Image frame.
        desired_center = tag_centers[tag_families.index(tagfamily)]
        delta_pixel_x = desired_center[0] - 320
        delta_pixel_y = desired_center[1] - 240
        alpha = np.abs(delta_pixel_x)*apx*np.pi/180
        beta = np.abs(delta_pixel_y)*apy*np.pi/180
        deltax_img = np.sign(delta_pixel_x)*np.tan(alpha)*tag_alt
        deltay_img = -np.sign(delta_pixel_y)*np.tan(beta)*tag_alt
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
        theta_vertical = np.arctan2(self.deltaS, tag_alt) #Angle to the target in relative to straight down plane

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
                rospy.loginfo_throttle(2,"\nHovering and aligning\n")
            #P controller to maintain altitude.
            if self.hover_alt is None:
                self.hover_alt =  self.actual_alt#Hover at 1.3 m.
            delta_alt = self.hover_alt - self.actual_alt
            gain_alt = 0.8
            self.set_uav_velocity.twist.linear.z = gain_alt*delta_alt
            
            # Set the linear velocity components in the x and y directions.
            self.set_uav_velocity.header.stamp = rospy.Time.now()
            self.set_uav_velocity.twist.linear.x = self.linear_vel * np.cos(theta_horizontal)
            self.set_uav_velocity.twist.linear.y = self.linear_vel * np.sin(theta_horizontal)

        else:
            if self.current_state.mode == "OFFBOARD":
                rospy.loginfo_throttle(2,"\nDescending\n")
            # Set the linear velocity components in the x and y directions.
            self.set_uav_velocity.header.stamp = rospy.Time.now()
            self.set_uav_velocity.twist.linear.x = self.linear_vel * np.cos(theta_horizontal)
            self.set_uav_velocity.twist.linear.y = self.linear_vel * np.sin(theta_horizontal)
            self.set_uav_velocity.twist.linear.z = -0.5 #Descend with 0.2 m/s.

            if self.hover_alt is not None:
                self.hover_alt = None
        
        # rospy.loginfo_throttle(2,f"\nlinearx_vel: {self.set_uav_velocity.twist.linear.x}\nlineary_vel: {self.set_uav_velocity.twist.linear.y}"+
        #               f"\nlinearz_vel: {self.set_uav_velocity.twist.linear.z}\n")

    def land(self):
        if self.land_time is None:
            self.land_time = rospy.Time.now().to_sec()
        self.proximity.append(self.deltaS)
        # rospy.loginfo(f"\nArray: {self.proximity}\n")
        if (rospy.Time.now().to_sec() - self.land_time) > 1:
            # rospy.loginfo(f"\nProximity: {np.mean(self.proximity)}\nActual Alt: {self.actual_alt}")
            if np.mean(self.proximity) < 0.1 and self.artag_alt_msg.altitude < 0.5:
                self.landing_condition = True 
            else:
                self.land_time = None
                self.proximity = []


def main():
    
    Ct = Controls()
    while not rospy.is_shutdown():

        tag_detected = Ct.artag_msg.detected

        if tag_detected:

            # Check Landing condition.
            Ct.land()
            
            # Initiate a timer.
            if WP.artag_detected_time is None:
                WP.artag_detected_time = rospy.Time.now().to_sec()

            # Start publishing required velocity to move towards the target.
            if "tag25h9" in WP.artag_family and WP.mainARTag_detected_time is None:
                WP.mainARTag_detected_time = rospy.Time.now().to_sec()

            if WP.current_state.mode != "OFFBOARD":
                if "tag36h11" not in WP.artag_family or ("tag25h9" in WP.artag_family \
                                and (rospy.Time.now().to_sec() - WP.mainARTag_detected_time) > 2):
                    WP.align(tagfamily="tag25h9")
                else:
                    WP.align()  

                # After few seconds of timer initiation, change the flight mode. 
                if (rospy.Time.now().to_sec() - WP.artag_detected_time) > 1 and WP.current_state.mode == "AUTO.LOITER":
                    mode = set_mode(custom_mode='OFFBOARD')
                    WP.artag_previously_detected = True
            else:
                if (rospy.Time.now().to_sec() - WP.artag_detected_time) > 1:
                    if "tag36h11" not in WP.artag_family or ("tag25h9" in WP.artag_family \
                                and (rospy.Time.now().to_sec() - WP.mainARTag_detected_time) > 2):
                        WP.align(tagfamily="tag25h9")
                    else:
                        WP.align()

            if WP.artag_lost_time is not None:
                    WP.artag_lost_time = None

        # If a artag is not detected at the target waypoint.        
        elif WP.targetWP_reached and (rospy.Time.now().to_sec() - WP.targetWP_reached_time) > 1 and WP.frame_updated and len(WP.artag_center) == 0:

            # Ascend if it was previously detected at the location.
            if WP.artag_previously_detected:
                if WP.artag_lost_time is None:
                    mode = set_mode(custom_mode='AUTO.LOITER')
                    if mode.mode_sent:
                        rospy.loginfo("\nTarget lost but previously detected. Loiter for 2 secs.\n")
                        WP.artag_lost_time = rospy.Time.now().to_sec()

                elif (rospy.Time.now().to_sec() - WP.artag_lost_time) > 1:
                    WP.set_uav_velocity.header.stamp = rospy.Time.now()
                    WP.set_uav_velocity.twist.linear.x = 0
                    WP.set_uav_velocity.twist.linear.y = 0
                    WP.set_uav_velocity.twist.linear.z = 0.3
                
                elif (rospy.Time.now().to_sec() - WP.artag_lost_time) > 2:
                    if WP.current_state.mode == 'AUTO.LOITER':
                        mode = set_mode(custom_mode='OFFBOARD')
                        if mode.mode_sent:
                            rospy.loginfo_throttle(2,"\nLost Target for more than 2 secs. Ascending.\n")
                    WP.set_uav_velocity.header.stamp = rospy.Time.now()
                    WP.set_uav_velocity.twist.linear.x = 0
                    WP.set_uav_velocity.twist.linear.y = 0
                    WP.set_uav_velocity.twist.linear.z = 0.3
                
                if WP.artag_detected_time is not None:
                    WP.artag_detected_time = None

            # Move to a different waypoint.
            else:
                rospy.loginfo(f"\nPushing wp:\n1. Lat - {WP.wamv_coordinate.latitude}\n2. Lon - {WP.wamv_coordinate.longitude + (np.random.rand(1) - 1)/10000}\n3. Alt - 6\n")
                WP.push_wp(push,pull,WP.wamv_coordinate.latitude,(WP.wamv_coordinate.longitude + (np.random.rand(1) - 1)/10000),6)
                mode = set_mode(custom_mode='AUTO.MISSION')
                if mode.mode_sent:
                    rospy.loginfo_throttle(2,"\nMoving to the next wp.\n")
                WP.targetWP_reached = False



        uav_vel_pub.publish(Ct.uav_vel_msg)
        landing_pub.publish(Ct.landing_condition)
        rate.sleep()