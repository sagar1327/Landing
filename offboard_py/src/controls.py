#!/usr/bin/env python3

import rospy
import numpy as np
from offboard_py.msg import ArTag, ArTagAltitude, MissionStatus, MissionName
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
        self.mission_status_msg = MissionStatus()

        self.deltaS = np.Inf
        self.land_time = None
        self.proximity = []
        self.angle = (0, 0, 0)
        self.hover_alt = None
        self.artag_lost = False
        self.targetWP_reached_time = None
        self.mainARTag_detected_time = None
        self.state_updated = False

        self.mission_status_msg.landing = True

        rospy.Subscriber("/kevin/artag", ArTag, callback=self.artag)
        rospy.Subscriber("/mavros/state", State, callback=self.uav_state)
        rospy.Subscriber("/kevin/artag/altitude", ArTagAltitude, callback=self.artag_alt)
        rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback=self.uav_pose)
        # rospy.Subscriber("/kevin/mission/status", MissionStatus, callback=self.mission_status)

        self.uav_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=3)
        self.rate = rospy.Rate(60)

        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def artag(self, msg):
        self.artag_msg = msg

    def artag_alt(self, msg):
        self.artag_alt_msg = msg

    def uav_state(self, msg):
        self.uav_state_msg = msg
        self.state_updated = True
    
    # def mission_status(self, msg):
    #     self.mission_status_msg = msg

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
            if np.mean(self.proximity) < 0.3 and self.artag_alt_msg.altitude < 0.8:
                return 1 
            else:
                self.land_time = None
                self.proximity = []
        return 0


def main():
    
    Ct = Controls()
    while not rospy.is_shutdown():

        # print(Ct.mission_status_msg)
        if Ct.mission_status_msg.landing:
            # if Ct.targetWP_reached_time is None:
            #     Ct.targetWP_reached_time = rospy.Time.now().to_sec()

            # if (rospy.Time.now().to_sec()-Ct.targetWP_reached_time) > 3:
                
            if Ct.artag_msg.detected:

                # Initiate a timer.
                if Ct.artag_detected_time is None:
                    Ct.artag_detected_time = rospy.Time.now().to_sec()

                ret = Ct.land()
                if ret:
                    mode = Ct.set_mode(custom_mode='AUTO.LAND')
                    if mode.mode_sent:
                        Ct.state_updated = False
                        print("Landing.")
                    break

                # Start publishing required velocity to move towards the target.
                if "tag25h9" in Ct.artag_msg.family_names and Ct.mainARTag_detected_time is None:
                    Ct.mainARTag_detected_time = rospy.Time.now().to_sec()

                if Ct.state_updated and Ct.uav_state_msg.mode != "OFFBOARD":
                    if (Ct.artag_msg.total_tags > 1 and (rospy.Time.now().to_sec() - Ct.mainARTag_detected_time) > 2) or "tag36h11" not in Ct.artag_msg.family_names:
                        Ct.align(tagfamily="tag25h9")
                    else:
                        Ct.align()  

                    # After few seconds of timer initialization, change the flight mode. 
                    if (rospy.Time.now().to_sec() - Ct.artag_detected_time) > 1:
                        # print("Setting OFFBOARD mode.")
                        mode = Ct.set_mode(custom_mode="OFFBOARD")
                        if mode.mode_sent:
                            print("Vehicle in OFFBOARD mode.")
                        # Ct.artag_previously_detected = True
                elif Ct.state_updated and Ct.uav_state_msg.mode == "OFFBOARD":
                    if (rospy.Time.now().to_sec() - Ct.artag_detected_time) > 1:
                        if (Ct.artag_msg.total_tags > 1 and (rospy.Time.now().to_sec() - Ct.mainARTag_detected_time) > 2) or "tag36h11" not in Ct.artag_msg.family_names:
                            Ct.align(tagfamily="tag25h9")
                        else:
                            Ct.align()

            # Ascend if it was previously detected at the location.
            elif not Ct.artag_msg.detected and Ct.artag_msg.previously_detected:
                # print(f"{Ct.artag_msg.duration}")
                if Ct.artag_detected_time is not None:
                    mode = Ct.set_mode(custom_mode='AUTO.LOITER')
                    if mode.mode_sent:
                        print("Target lost but previously detected. Loiter for 1 secs.")
                    Ct.artag_detected_time = None

                if Ct.artag_msg.duration > 0.5 and Ct.artag_msg.duration < 1:
                    Ct.uav_vel_msg.header.stamp = rospy.Time.now()
                    Ct.uav_vel_msg.twist.linear.x = 0
                    Ct.uav_vel_msg.twist.linear.y = 0
                    Ct.uav_vel_msg.twist.linear.z = 0.3         
                elif Ct.artag_msg.duration > 1:
                    if Ct.uav_state_msg.mode != "OFFBOARD":
                        mode = Ct.set_mode(custom_mode='OFFBOARD')
                        if mode.mode_sent:
                            print("Lost Target for more than 1 secs. Ascending.")
                    Ct.uav_vel_msg.header.stamp = rospy.Time.now()
                    Ct.uav_vel_msg.twist.linear.x = 0
                    Ct.uav_vel_msg.twist.linear.y = 0
                    Ct.uav_vel_msg.twist.linear.z = 0.3

            # else:
            #     print("Waiting")            
        
        
        Ct.uav_vel_pub.publish(Ct.uav_vel_msg)
        Ct.state_updated = False
        Ct.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass