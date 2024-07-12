#!/usr/bin/env python3

import rospy
import numpy as np
from offboard_py.msg import ArTag, ArTagAltitude, MissionStatus
from mavros_msgs.msg import State, WaypointReached
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import  TwistStamped, PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool


class Controls():

    def __init__(self):

        rospy.init_node("uav_controls", anonymous=True)

        self.artag_msg = ArTag()
        self.artag_alt_msg = ArTagAltitude()
        self.artag_detected_time = None
        self.uav_state_msg = State()
        self.uav_vel_msg = TwistStamped()
        self.uav_vel_msg.header.frame_id = 'map'
        self.current_pose = PoseStamped()
        self.wp_reached_msg = WaypointReached()
        self.landing_condition_msg = Bool()
        self.landing_condition_msg.data = False
        self.current_state_msg = State()
        self.mission_status_msg = MissionStatus()

        self.deltaS = np.Inf
        self.land_time = None
        self.proximity = []
        self.landing_condition = False
        self.angle = (0, 0, 0)
        self.hover_alt = None
        self.artag_lost_time = None

        rospy.Subscriber("/kevin/artag", ArTag, callback=self.artag)
        rospy.Subscriber("/mavros/state", State, callback=self.uav_state)
        rospy.Subscriber("/kevin/artag/altitude", ArTagAltitude, callback=self.artag_alt)
        rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback=self.uav_pose)
        rospy.Subscriber("/mavros/mission/reached", WaypointReached, callback=self.wp_reached)
        rospy.Subscriber('mavros/state', State, callback=self.current_state)

        self.uav_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=3)
        self.landing_pub = rospy.Publisher("/kevin/landing", Bool, queue_size=1)
        self.new_mission_pub = rospy.Publisher("/kevin/mission/status", MissionStatus, queue_size=1)
        self.rate = rospy.Rate(60)

        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def artag(self, msg):
        self.artag_msg = msg

    def artag_alt(self, msg):
        self.artag_alt_msg = msg

    def uav_state(self, msg):
        self.uav_state_msg = msg

    def wp_reached(self, msg):
        self.wp_reached_msg = msg

    def current_state(self, msg):
        self.current_state_msg = msg

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
        delta_pixel_x = desired_center.x - 320
        delta_pixel_y = desired_center.y - 240
        alpha = np.abs(delta_pixel_x)*apx*np.pi/180
        beta = np.abs(delta_pixel_y)*apy*np.pi/180
        deltax_img = np.sign(delta_pixel_x)*np.tan(alpha)*tag_alt
        deltay_img = -np.sign(delta_pixel_y)*np.tan(beta)*tag_alt
        deltaS_img = np.sqrt(np.square(deltax_img) + np.square(deltay_img))
        theta_img = np.arctan2(deltay_img, deltax_img)

        # #Global frame.
        self.deltaS = deltaS_img
        # theta = theta_img+self.angle[2]-np.pi/2

        # linear_vel = np.sqrt(2 * 0.01 * self.deltaS)
        # self.uav_vel_msg.header.frame_id = 'map'
        # self.uav_vel_msg.header.stamp = rospy.Time.now()
        # self.uav_vel_msg.twist.linear.x = linear_vel * np.cos(theta)
        # self.uav_vel_msg.twist.linear.y = linear_vel * np.sin(theta)
        # self.uav_vel_msg.twist.linear.z = 0.0

        # rospy.loginfo(f"\napx: {apx}\napy: {apy}\nARTag center: {self.artag_center}\ndelta_pixel_x: {delta_pixel_x}"+
        #               f"\ndelta_pixel_y: {delta_pixel_y}\nalpha: {alpha}\nbeta: {beta}\nCurrent height: {self.current_pose.pose.position.z}\ndeltax_img: {deltax_img}\ndeltay_img: {deltay_img}"+
        #               f"\ndeltaS_img: {deltaS_img}\ntheta_img: {theta_img}\nHeading: {self.angle[2]}\ntheta: {theta}"+
        #               f"\nlinear_vel: {linear_vel}\nlinearx_vel: {self.uav_vel_msg.twist.linear.x}\nlineary_vel: {self.uav_vel_msg.twist.linear.y}\n")

        # Calculate the angle between the estimated position and the target position
        theta_horizontal = theta_img+self.angle[2]-np.pi/2 #Angle to the target in x-y plane
        theta_vertical = np.arctan2(self.deltaS, tag_alt) #Angle to the target in relative to straight down plane

        # Calculate the linear velocity based on the distance.
        if tag_alt > 0.5:
            gain = 0.3
        else:
            gain = 1
        linear_vel = gain * self.deltaS
        # linear_vel = 0.8 if linear_vel > 0.8 else linear_vel #Avoid going too fast to be safe was 0.5.
        ##################################
        desired_heading = np.pi/2
        current_heading = self.angle[2]
        gain_heading = 0.2
        angular_z_vel = gain_heading*(desired_heading-current_heading)
        self.uav_vel_msg.twist.angular.z = angular_z_vel

        # Publish the correction position if the distance is greater than 0.2.
        if theta_vertical > 8*np.pi/180:
            if self.uav_state_msg.mode == "OFFBOARD":
                rospy.loginfo_throttle(2,"\nHovering and aligning\n")
            #P controller to maintain altitude.
            if self.hover_alt is None:
                self.hover_alt =  tag_alt#Hover at 1.3 m.
            delta_alt = self.hover_alt - tag_alt
            gain_alt = 0.8
            self.uav_vel_msg.twist.linear.z = gain_alt*delta_alt
            
            # Set the linear velocity components in the x and y directions.
            self.uav_vel_msg.header.stamp = rospy.Time.now()
            self.uav_vel_msg.twist.linear.x = linear_vel * np.cos(theta_horizontal)
            self.uav_vel_msg.twist.linear.y = linear_vel * np.sin(theta_horizontal)

        else:
            if self.uav_state_msg.mode == "OFFBOARD":
                rospy.loginfo_throttle(2,"\nDescending\n")
            # Set the linear velocity components in the x and y directions.
            self.uav_vel_msg.header.stamp = rospy.Time.now()
            self.uav_vel_msg.twist.linear.x = linear_vel * np.cos(theta_horizontal)
            self.uav_vel_msg.twist.linear.y = linear_vel * np.sin(theta_horizontal)
            self.uav_vel_msg.twist.linear.z = -0.5 #Descend with 0.2 m/s.

            if self.hover_alt is not None:
                self.hover_alt = None
        
        # rospy.loginfo_throttle(2,f"\nlinearx_vel: {self.uav_vel_msg.twist.linear.x}\nlineary_vel: {self.uav_vel_msg.twist.linear.y}"+
        #               f"\nlinearz_vel: {self.uav_vel_msg.twist.linear.z}\n")

    def land(self):
        if self.land_time is None:
            self.land_time = rospy.Time.now().to_sec()
        self.proximity.append(self.deltaS)
        # rospy.loginfo(f"\nArray: {self.proximity}\n")
        if (rospy.Time.now().to_sec() - self.land_time) > 0.3:
            # rospy.loginfo(f"\nProximity: {np.mean(self.proximity)}\nActual Alt: {self.actual_alt}")
            print(f"{np.mean(self.proximity)} {self.artag_alt_msg.altitude}")
            if np.mean(self.proximity) < 0.3 and self.artag_alt_msg.altitude < 1:
                return 1 
            else:
                self.land_time = None
                self.proximity = []
        return 0


def main():
    
    Ct = Controls()
    while not rospy.is_shutdown():

        tag_detected = Ct.artag_msg.detected

        if tag_detected:

            # Initiate a timer.
            if Ct.artag_detected_time is None:
                Ct.artag_detected_time = rospy.Time.now().to_sec()

            Ct.align()
            
            if (rospy.Time.now().to_sec() - Ct.artag_detected_time) > 1:

                # Check Landing condition.
                landing_condition = Ct.land()
                if landing_condition:
                    Ct.landing_condition_msg.data = True
                    Ct.landing_pub.publish(Ct.landing_condition_msg)
                    mode = Ct.set_mode(custom_mode='OFFBOARD')
                    if mode.mode_sent:
                        print("Vehicle now landing.")
                    break

                if Ct.current_state_msg.mode == "AUTO.LOITER":
                    mode = Ct.set_mode(custom_mode='OFFBOARD')
                    if mode.mode_sent:
                        print("Vehicle in offboard mode.")
        else:
            if Ct.artag_msg.previously_detected:
                if Ct.artag_lost_time is None:
                    mode = Ct.set_mode(custom_mode='AUTO.LOITER')
                    if mode.mode_sent:
                        rospy.loginfo("\nTarget lost but previously detected. Loiter for 1 secs.\n")
                        Ct.artag_lost_time = rospy.Time.now().to_sec()

                elif (rospy.Time.now().to_sec() - Ct.artag_lost_time) > 0.5:
                    Ct.uav_vel_msg.header.stamp = rospy.Time.now()
                    Ct.uav_vel_msg.twist.linear.x = 0
                    Ct.uav_vel_msg.twist.linear.y = 0
                    Ct.uav_vel_msg.twist.linear.z = 0.3
                
                elif (rospy.Time.now().to_sec() - Ct.artag_lost_time) > 1:
                    mode = Ct.set_mode(custom_mode='OFFBOARD')
                    if mode.mode_sent:
                        rospy.loginfo_throttle(2,"\nLost Target for more than 1 secs. Ascending.\n")
                    Ct.uav_vel_msg.header.stamp = rospy.Time.now()
                    Ct.uav_vel_msg.twist.linear.x = 0
                    Ct.uav_vel_msg.twist.linear.y = 0
                    Ct.uav_vel_msg.twist.linear.z = 0.3
            else:
                if not Ct.mission_status_msg.new_mission_pushed:

                    Ct.mission_status_msg.header.stamp = rospy.Time.now()
                    Ct.mission_status_msg.header.frame_id = 'map'
                    Ct.mission_status_msg.new_mission_request = True
                    Ct.mission_status_msg.new_mission_pushed = False
                    Ct.mission_status_msg.new_mission_pulled = False

                elif Ct.current_state_msg.mode == "AUTO.MISSION":

                    Ct.mission_status_msg.header.stamp = rospy.Time.now()
                    Ct.mission_status_msg.header.frame_id = 'map'
                    Ct.mission_status_msg.new_mission_request = False
                    Ct.mission_status_msg.new_mission_pushed = True
                    Ct.mission_status_msg.new_mission_pulled = True

        #     # Start publishing required velocity to move towards the target.
        #     if "tag25h9" in Ct.artag_msg.family_names and Ct.mainARTag_detected_time is None:
        #         Ct.mainARTag_detected_time = rospy.Time.now().to_sec()

        #     if Ct.current_state.mode != "OFFBOARD":
        #         if "tag36h11" not in Ct.artag_msg.family_names or ("tag25h9" in Ct.artag_msg.family_names \
        #                         and (rospy.Time.now().to_sec() - Ct.mainARTag_detected_time) > 2):
        #             Ct.align(tagfamily="tag25h9")
        #         else:
        #             Ct.align()  

        #         # After few seconds of timer initiation, change the flight mode. 
        #         if (rospy.Time.now().to_sec() - Ct.artag_detected_time) > 1 and Ct.current_state.mode == "AUTO.LOITER":
        #             mode = Ct.set_mode(custom_mode='OFFBOARD')
        #             Ct.artag_previously_detected = True
        #     else:
        #         if (rospy.Time.now().to_sec() - Ct.artag_detected_time) > 1:
        #             if "tag36h11" not in Ct.artag_family or ("tag25h9" in Ct.artag_family \
        #                         and (rospy.Time.now().to_sec() - Ct.mainARTag_detected_time) > 2):
        #                 Ct.align(tagfamily="tag25h9")
        #             else:
        #                 Ct.align()

        #     if Ct.artag_lost_time is not None:
        #             Ct.artag_lost_time = None

        # # If a artag is not detected at the target waypoint.        
        # elif Ct.targetCt_reached and (rospy.Time.now().to_sec() - Ct.targetCt_reached_time) > 1 and Ct.frame_updated and len(Ct.artag_center) == 0:

        #     # Ascend if it was previously detected at the location.
        #     if Ct.artag_previously_detected:
        #         if Ct.artag_lost_time is None:
        #             mode = set_mode(custom_mode='AUTO.LOITER')
        #             if mode.mode_sent:
        #                 rospy.loginfo("\nTarget lost but previously detected. Loiter for 2 secs.\n")
        #                 Ct.artag_lost_time = rospy.Time.now().to_sec()

        #         elif (rospy.Time.now().to_sec() - Ct.artag_lost_time) > 1:
        #             Ct.uav_vel_msg.header.stamp = rospy.Time.now()
        #             Ct.uav_vel_msg.twist.linear.x = 0
        #             Ct.uav_vel_msg.twist.linear.y = 0
        #             Ct.uav_vel_msg.twist.linear.z = 0.3
                
        #         elif (rospy.Time.now().to_sec() - Ct.artag_lost_time) > 2:
        #             if Ct.current_state.mode == 'AUTO.LOITER':
        #                 mode = set_mode(custom_mode='OFFBOARD')
        #                 if mode.mode_sent:
        #                     rospy.loginfo_throttle(2,"\nLost Target for more than 2 secs. Ascending.\n")
        #             Ct.uav_vel_msg.header.stamp = rospy.Time.now()
        #             Ct.uav_vel_msg.twist.linear.x = 0
        #             Ct.uav_vel_msg.twist.linear.y = 0
        #             Ct.uav_vel_msg.twist.linear.z = 0.3
                
        #         if Ct.artag_detected_time is not None:
        #             Ct.artag_detected_time = None

        #     # Move to a different waypoint.
        #     else:
        #         rospy.loginfo(f"\nPushing Ct:\n1. Lat - {Ct.wamv_coordinate.latitude}\n2. Lon - {Ct.wamv_coordinate.longitude + (np.random.rand(1) - 1)/10000}\n3. Alt - 6\n")
        #         Ct.push_Ct(push,pull,Ct.wamv_coordinate.latitude,(Ct.wamv_coordinate.longitude + (np.random.rand(1) - 1)/10000),6)
        #         mode = set_mode(custom_mode='AUTO.MISSION')
        #         if mode.mode_sent:
        #             rospy.loginfo_throttle(2,"\nMoving to the next Ct.\n")
        #         Ct.targetCt_reached = False


        Ct.new_mission_pub.publish(Ct.mission_status_msg)
        Ct.uav_vel_pub.publish(Ct.uav_vel_msg)
        Ct.landing_pub.publish(Ct.landing_condition_msg)
        Ct.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass