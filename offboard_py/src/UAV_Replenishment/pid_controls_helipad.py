#!/usr/bin/env python3

import rospy
import numpy as np
## *** from offboard_py.msg import ArTag, ArTagAltitude, MissionStatus, SetPoint # *** TODO: from offboard_py.msg import Helipad msg, SquareCenters msg, SqaureAltitude msg, ConcCenters msg, ConcCircAltitude msg, TinCenters msg, TinAltitude msg, MissionStatus, SetPoint ***
from offboard_py.msg import * ## Helipad, HelipadAltitude, SquareCenters, TinCenters, TinAltitude, MissionStatus, SetPoint
from mavros_msgs.msg import State, WaypointReached
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import  TwistStamped, PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool, String, Float64


class Controls():

    def __init__(self):

        rospy.init_node("uav_controls", anonymous=True)

        self.helipad_msg = Helipad() ## *********** 8===========D
        self.helipad_alt_msg = Float64() ## *********** 8===========D
        self.helipad_detected_time = None ## *********** 8===========D
        
        ## self.tin_msg = Centers() ## *********** 8===========D
        ## self.tin_alt_msg = TinAltitude()
        ## self.tin_detected_time = None ## *********** 8===========D
                
        #self.artag_msg = ArTag() ## *******
        #self.artag_alt_msg = ArTagAltitude() ## *******
        #self.artag_detected_time = None ## *****
        
        self.uav_state_msg = State()
        self.uav_vel_msg = TwistStamped()
        self.uav_vel_msg.header.frame_id = 'map'
        self.current_pose = PoseStamped()
        ## self.land_on_boat_msg = Bool() ## *********** 8===========D
        ## self.setpoint = SetPoint() ## *********** 8===========D
        self.landing_seq_msg = String()

        self.current_deltaS = np.Inf
        # self.previous_deltaS = 0.0
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_time = None
        # self.integral = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.land_time = None
        self.proximity = []
        self.angle = (0, 0, 0)
        self.hover_alt = None
        ## ****** self.artag_lost = False
        self.helipad_lost = False ## *********** 8===========D
        self.targetWP_reached_time = None
        ## ****** self.mainARTag_detected_time = None # main
        ## self.mainHelipad_detected_time = None # main ## *********** 8===========D
        self.state_updated = False
        self.landing_sequences = ["Hovering and Aligning", "Descending", "Holding", "Landing"]
        self.current_seq = None
        ## self.previous_tag_family = None ## *********** 8===========D

        #rospy.Subscriber("/kevin/artag/info", ArTag, callback=self.artag)
        #rospy.Subscriber("/mavros/state", State, callback=self.uav_state)
        #rospy.Subscriber("/kevin/artag/altitude", ArTagAltitude, callback=self.artag_alt)
        #rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback=self.uav_pose)
        #rospy.Subscriber("/kevin/land_permission", Bool, callback=self.landing_status)
        
        rospy.Subscriber("/kevin/target/squares/center", Center, callback=self.helipad_center) ## *********** 8===========D
        rospy.Subscriber("/mavros/state", State, callback=self.uav_state)
        ## rospy.Subscriber("/kevin/helipad/altitude", Float64, callback=self.helipad_alt) ## *********** 8===========D
        rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback=self.uav_pose)
        ## rospy.Subscriber("/kevin/land_permission", Bool, callback=self.landing_status) ## *********** 8===========D

        self.uav_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=3)
        ## self.setpoint_pub = rospy.Publisher("/kevin/pid/setpoint", SetPoint, queue_size=1) ## *********** 8===========D
        self.landing_seq_pub = rospy.Publisher("/kevin/landing/sequence", String, queue_size=1)
        self.rate = rospy.Rate(60)

        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    ## ***** def artag(self, msg):
        ## ***** self.artag_msg = msg

    ## ***** def artag_alt(self, msg):
        ## ***** self.artag_alt_msg = msg
        
    def helipad_center(self, msg): ## *********** 8===========D
        self.helipad_msg.center = msg ## *********** 8===========D
        ## TODO: fill msg for corners

    def helipad_alt(self, msg): ## *********** 8===========D NOT CALLED
        self.helipad_alt_msg = msg ## *********** 8===========D

    def uav_state(self, msg):
        self.uav_state_msg = msg
        self.state_updated = True
    
    ## def landing_status(self, msg): ## *********** 8===========D
        ## self.land_on_boat_msg = msg ## *********** 8===========D

    def uav_pose(self, msg):
        self.current_pose = msg
        self.angle = euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])

    ## ***** def align(self, tagfamily="tag36h11"):
    def align(self):                   ## *********** 8===========D
        ## current_tag_family = tagfamily
        apx = 65/640
        apy = 65/480
        ## tag_centers = self.artag_msg.centers    ###### ??????????????
        ## tag_families = self.artag_msg.family_names   ####### ????????????????????
        ## tag_alt = self.artag_alt_msg.altitude     ############
        tag_alt = self.helipad_alt_msg.data

        # Image frame.
        ## desired_center = tag_centers[tag_families.index(tagfamily)]    ########### ???????????????????
        ## delta_pixel_x = desired_center.x - 320
        ## delta_pixel_y = desired_center.y - 240
        
        # TODO: find x-val, find y-val, then define delta_pixel_x & delta_pixel_y accordingly     ## *********** 8===========D
        delta_pixel_x = self.helipad_msg.center.x - 320      ## *********** 8===========D 
        delta_pixel_y = self.helipad_msg.center.y - 240          ## *********** 8===========D 
        alpha = np.abs(delta_pixel_x)*apx*np.pi/180
        beta = np.abs(delta_pixel_y)*apy*np.pi/180
        
        ## deltax_img = np.sign(delta_pixel_x)*np.tan(alpha)*self.helipad_alt_msg.data ## *********** 8===========D TODO: uncomment `rospy.Subscriber("/kevin/helipad/altitude", Float64, callback=self.helipad_alt)`, REQUIRES height/altitude estimate from square_detect.py
        ## deltay_img = -np.sign(delta_pixel_y)*np.tan(beta)*self.helipad_alt_msg.data ## *********** 8===========D TODO: uncomment `rospy.Subscriber("/kevin/helipad/altitude", Float64, callback=self.helipad_alt)`, REQUIRES height/altitude estimate from square_detect.py
        deltax_img = np.sign(delta_pixel_x)*np.tan(alpha)*tag_alt ## *********** 8===========D units in meters ---> comment out 
        deltay_img = -np.sign(delta_pixel_y)*np.tan(beta)*tag_alt ## *********** 8===========D units in meters ---> comment out 
        deltaS_img = np.sqrt(np.square(deltax_img) + np.square(deltay_img))
        theta_img = np.arctan2(deltay_img, deltax_img)

        # Global frame.
        self.current_deltaS = deltaS_img

        ## if current_tag_family != self.previous_tag_family: ## *********** 8===========D
           ## self.setpoint = SetPoint()                   ## *********** 8===========D
        ## self.previous_tag_family = current_tag_family ## *********** 8===========D

        # Calculate the angle between the estimated position and the target position
        theta_horizontal = theta_img+self.angle[2]-np.pi/2 #Angle to the target in x-y plane
        theta_vertical = np.arctan2(self.current_deltaS, tag_alt) #Angle to the target in relative to straight down plane

        ## if self.setpoint.setpoint.x == 0 and self.setpoint.setpoint.y == 0: ## *********** 8===========D
            ## self.setpoint.setpoint.x = self.current_deltaS*np.cos(theta_horizontal) ## *********** 8===========D
            ## self.setpoint.setpoint.y = self.current_deltaS*np.sin(theta_horizontal) ## *********** 8===========D

        # PD controller
        # kp = 1;ki = 0.0;kd = 0.1
        # linear_vel = kp*self.current_deltaS
        kpx = 0.5;kix = 0.0;kdx = 0.0 #kix = 0.0;kdx = 0.08
        kpy = 0.5;kiy = 0.0;kdy = 0.0 #kiy = 0.08;kdy = 0.06
        deltax = self.current_deltaS*np.cos(theta_horizontal)
        deltay = self.current_deltaS*np.sin(theta_horizontal)
        linear_vel_x = kpx*deltax
        linear_vel_y = kpy*deltay

        desired_heading = np.pi/2
        current_heading = self.angle[2]
        gain_heading = 0.2
        angular_z_vel = gain_heading*(desired_heading-current_heading)
        self.uav_vel_msg.twist.angular.z = angular_z_vel

        if theta_vertical <= 5*np.pi/180: #and tag_alt > 3:
            if self.state_updated and self.uav_state_msg.mode == "OFFBOARD":
                self.current_seq = self.landing_sequences[1]
                self.landing_seq_msg.data = self.current_seq
                rospy.loginfo_throttle(2,self.current_seq)

                # if self.setpoint.setpoint.x == 0 and self.setpoint.setpoint.y == 0:
                #     self.setpoint.setpoint.x = self.current_deltaS*np.cos(theta_horizontal)
                #     self.setpoint.setpoint.y = self.current_deltaS*np.sin(theta_horizontal)

                if self.previous_time is None:
                    self.previous_time = rospy.Time.now().to_sec()
                current_time = rospy.Time.now().to_sec()
                dt = current_time - self.previous_time
                # self.integral += self.current_deltaS
                # derivative = (self.current_deltaS - self.previous_deltaS)/dt if dt > 0 else 0
                self.integral_x += deltax*dt
                self.integral_y += deltay*dt
                derivative_x = (deltax - self.previous_x)/dt if dt > 0 else 0
                derivative_y = (deltay - self.previous_y)/dt if dt > 0 else 0

                linear_vel_x = linear_vel_x + kix*self.integral_x + kdx*derivative_x
                linear_vel_y = linear_vel_y + kiy*self.integral_y + kdy*derivative_y
                self.previous_x = deltax
                self.previous_y = deltay
                self.previous_time = current_time
                # linear_vel = linear_vel + ki*self.integral + kd*derivative
                # self.previous_deltaS = self.current_deltaS
                # self.previous_time = current_time

            
            # print(f"Descending")
            # print(f"Linear x: {linear_vel_x} and Linear y: {linear_vel_y}")
            # Set the linear velocity components in the x and y directions.
            self.uav_vel_msg.header.stamp = rospy.Time.now()
            self.uav_vel_msg.twist.linear.x = linear_vel_x
            self.uav_vel_msg.twist.linear.y = linear_vel_y
            self.uav_vel_msg.twist.linear.z = -0.3 #Descend with 0.2 m/s.

            if self.hover_alt is not None:
                self.hover_alt = None

            # self.setpoint = SetPoint()
        
        else:
            if self.state_updated and self.uav_state_msg.mode == "OFFBOARD":
                self.current_seq = self.landing_sequences[0]
                self.landing_seq_msg.data = self.current_seq
                rospy.loginfo_throttle(2,self.current_seq)

                # if self.setpoint.setpoint.x == 0 and self.setpoint.setpoint.y == 0:
                #     self.setpoint.setpoint.x = self.current_deltaS*np.cos(theta_horizontal)
                #     self.setpoint.setpoint.y = self.current_deltaS*np.sin(theta_horizontal)

                if self.previous_time is None:
                    self.previous_time = rospy.Time.now().to_sec()
                current_time = rospy.Time.now().to_sec()
                dt = current_time - self.previous_time
                # self.integral += self.current_deltaS
                # derivative = (self.current_deltaS - self.previous_deltaS)/dt if dt > 0 else 0
                self.integral_x += deltax*dt
                self.integral_y += deltay*dt
                derivative_x = (deltax - self.previous_x)/dt if dt > 0 else 0
                derivative_y = (deltay - self.previous_y)/dt if dt > 0 else 0

                linear_vel_x = linear_vel_x + kix*self.integral_x + kdx*derivative_x
                linear_vel_y = linear_vel_y + kiy*self.integral_y + kdy*derivative_y
                self.previous_x = deltax
                self.previous_y = deltay
                self.previous_time = current_time
                # linear_vel = linear_vel + ki*self.integral + kd*derivative
                # self.previous_deltaS = self.current_deltaS
                # self.previous_time = current_time
            
                # P controller to maintain altitude.
                if self.hover_alt is None:
                    self.hover_alt =  tag_alt#Hover at 1.3 m.
                delta_alt = self.hover_alt - tag_alt
                print(f"Hovering at {tag_alt}. Need to move {delta_alt}")
                # print(f"Linear x: {linear_vel_x} and Linear y: {linear_vel_y}")
                gain_alt = 0.5
                self.uav_vel_msg.twist.linear.z = gain_alt*delta_alt
                
                # Set the linear velocity components in the x and y directions.
                self.uav_vel_msg.header.stamp = rospy.Time.now()
                self.uav_vel_msg.twist.linear.x = linear_vel_x
                self.uav_vel_msg.twist.linear.y = linear_vel_y

                # rospy.loginfo_throttle(2,f"\nlinearx_vel: {self.uav_vel_msg.twist.linear.x}\nlineary_vel: {self.uav_vel_msg.twist.linear.y}"+
                #               f"\nlinearz_vel: {self.uav_vel_msg.twist.linear.z}\n")

    def land(self):
        if self.land_time is None:
            self.land_time = rospy.Time.now().to_sec()
        self.proximity.append(self.current_deltaS)
        # rospy.loginfo(f"\nArray: {self.proximity}\n")
        if (rospy.Time.now().to_sec() - self.land_time) > 0.6:
            # rospy.loginfo(f"\nProximity: {np.mean(self.proximity)}\nActual Alt: {self.actual_alt}")
            print(f"{np.mean(self.proximity)} {self.helipad_alt_msg.altitude}") ## *********** 8===========D
            if np.mean(self.proximity) < 0.1 and self.helipad_alt_msg.altitude < 0.3: ## *********** 8===========D
                return 1 
            else:
                self.land_time = None
                self.proximity = []
        return 0


def main():
    
    Ct = Controls()
    while not rospy.is_shutdown():

        # print(Ct.land_on_boat_msg.data)

        # print(f"Detected: {Ct.helipad_msg.detected}")    
        if Ct.helipad_msg.detected:

            ## # Initiate a timer.
            if Ct.helipad_detected_time is None: ## *********** 8===========D
                Ct.helipad_detected_time = rospy.Time.now().to_sec() ## *********** 8===========D

            ret = Ct.land()
            if ret:
                mode = Ct.set_mode(custom_mode='AUTO.LAND')
                if mode.mode_sent:
                    Ct.state_updated = False
                    Ct.current_seq = Ct.landing_sequences[3]
                    Ct.landing_seq_msg.data = Ct.current_seq
                    print(Ct.current_seq)
                break

            # Start publishing required velocity to move towards the target.
            # if "tag25h9" in Ct.artag_msg.family_names and Ct.mainARTag_detected_time is None:
            #     Ct.mainARTag_detected_time = rospy.Time.now().to_sec()

            if Ct.state_updated and Ct.uav_state_msg.mode != "OFFBOARD":
                # if (Ct.artag_msg.total_tags > 1 and (rospy.Time.now().to_sec() - Ct.mainARTag_detected_time) > 2) ## or "tag36h11" not in Ct.artag_msg.family_names: ## *********** 8===========D
                #     Ct.align(tagfamily="tag25h9") ## *********** 8===========D
                #     # Ct.align() ## *********** 8===========D
                # else: ## *********** 8===========D
                #     Ct.align()  ## *********** 8===========D

                # After few seconds of timer initialization, change the flight mode. 
                if (rospy.Time.now().to_sec() - Ct.helipad_detected_time) > 2:
                    # print("Setting OFFBOARD mode.")
                    mode = Ct.set_mode(custom_mode="OFFBOARD")
                    if mode.mode_sent:
                        print("Vehicle in OFFBOARD mode.")
                    Ct.helipad_previously_detected = True
            elif Ct.state_updated and Ct.uav_state_msg.mode == "OFFBOARD":
                # if (rospy.Time.now().to_sec() - Ct.artag_detected_time) > 1:
                #     if (Ct.artag_msg.total_tags > 1 and (rospy.Time.now().to_sec() - Ct.mainARTag_detected_time) > 2) or "tag36h11" not in Ct.artag_msg.family_names:
                #         Ct.align(tagfamily="tag25h9")
                #         # Ct.align()
                #     else:
                Ct.align()

        # Ascend if it was previously detected at the location.
        elif Ct.state_updated and not Ct.current_seq == "Hovering and Aligning" and not Ct.helipad_msg.detected and Ct.helipad_msg.previously_detected: ## *********** 8===========D
            # print(f"{Ct.artag_msg.duration}")
            if Ct.helipad_detected_time is not None:
                mode = Ct.set_mode(custom_mode='AUTO.LOITER')
                if mode.mode_sent:
                    print("Target lost but previously detected. Loiter for 2 secs.")
                Ct.helipad_detected_time = None

            if Ct.helipad_msg.duration > 1 and Ct.helipad_msg.duration < 2:
                Ct.uav_vel_msg.header.stamp = rospy.Time.now()
                Ct.uav_vel_msg.twist.linear.x = 0
                Ct.uav_vel_msg.twist.linear.y = 0
                ## Ct.uav_vel_msg.twist.linear.z = 0.3        ## *********** 8===========D
                Ct.uav_vel_msg.twist.linear.z = 0 ## *********** 8===========D

            elif Ct.helipad_msg.duration > 2:
                if Ct.state_updated and Ct.uav_state_msg.mode != "OFFBOARD":
                    mode = Ct.set_mode(custom_mode='OFFBOARD')
                    if mode.mode_sent:
                        print("Lost Target for more than 2 secs. Ascending.")
                Ct.uav_vel_msg.header.stamp = rospy.Time.now()
                Ct.uav_vel_msg.twist.linear.x = 0
                Ct.uav_vel_msg.twist.linear.y = 0
                Ct.uav_vel_msg.twist.linear.z = 0.3

                ## Ct.setpoint = SetPoint() ## *********** 8===========D

            # else:
            #     print("Waiting")            
        
        
        Ct.uav_vel_pub.publish(Ct.uav_vel_msg)
        ## Ct.setpoint_pub.publish(Ct.setpoint) ## *********** 8===========D
        Ct.landing_seq_pub.publish(Ct.landing_seq_msg)
        Ct.state_updated = False
        Ct.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass