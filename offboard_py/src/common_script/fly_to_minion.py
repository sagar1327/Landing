#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, WaypointPull, SetMode
from offboard_py.msg import MissionStatus, ArTag


class FlyToMinion():
    """A node to make the drone fly to a waypoint.
       Requires: 1) waypoint coordinates. 2) Permission to fly.
       Outputs: True if reached minion (waypoint) and false otherwise."""
    def __init__(self):

        # rospy.init_node("Fly_to_wp",anonymous=True) ## *********** 
        rospy.init_node("Fly_to_minion",anonymous=True) ## 8====================D
        self.uav_state_msg = State()
        self.single_wp = Waypoint()

        # self.flyToWp_msg = Bool() ## ****************
        # self.flyToWp_msg.data = False ## ***************
        self.flyToMinion_msg = Bool()  ## 8====================D
        self.flyToMinion_msg.data = False  ## 8====================D

        self.wp_reached = Bool()
        self.wp_reached.data = False

        self.single_wp.frame = 3
        self.single_wp.command = 16
        self.single_wp.autocontinue = True
        self.single_wp.is_current = False
        self.single_wp.param1 = 0.0
        self.single_wp.param2 = 0.0
        self.single_wp.param3 = 0.0
        self.single_wp.param4 = float('nan')
        self.wp_pushed = False

        self.search_wps = [0,0]
        self.wp_pushed = False
        # self.new_wp_received = False
        self.minion_wp_received = False ## 8====================D
        self.state_updated = False
        
        rospy.wait_for_service("/mavros/mission/pull")
        self.pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull, persistent=True)
        rospy.wait_for_service("/mavros/mission/push")
        self.push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush, persistent=True)
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        ## rospy.Subscriber("/minion/kevin/target_wp", NavSatFix, callback=self.waypoint)
        rospy.Subscriber("/minion/sensors/pinpoint/fix", NavSatFix, callback=self.waypoint)  ## 8====================D
        rospy.Subscriber('mavros/state', State, callback=self.uav_state)
        ## rospy.Subscriber("/minion/kevin/fly_to_wp", Bool, callback=self.flyToWp)
        rospy.Subscriber("/minion/kevin/landing_permission", Bool, callback=self.flyToMinion)  ## 8====================D

        self.wp_status_pub = rospy.Publisher("/kevin/waypoint_reached", Bool, queue_size=1)
        self.rate = rospy.Rate(5)

    def waypoint(self, msg):

        if np.abs(msg.latitude - self.search_wps[0]) > 0.000009 and np.abs(msg.longitude - self.search_wps[1]) > 0.000009:
            self.search_wps = [msg.latitude, msg.longitude]
            # self.new_wp_received = True
            self.minion_wp_received = True  ## 8====================D

    def uav_state(self, msg):
        self.uav_state_msg = msg
        self.state_updated = True

    # def flyToWp(self, msg):
    #     self.flyToWp_msg = msg
    def flyToMinion(self, msg):  ## 8====================D
        self.flyToMinion_msg = msg  ## 8====================D


    def push_wp(self,lat,lon,alt):
        self.single_wp.x_lat =  lat
        self.single_wp.y_long = lon
        self.single_wp.z_alt = alt

        target_wp = [self.single_wp]

        # Push waypoints.
        try:
            self.push(start_index=0, waypoints=target_wp)
            print("Waypoint pushed.")
        except rospy.ServiceException as e:
            print("Service call failed: %s", e)
            return 0

        # Pull waypoints.
        try:
            wp_count = self.pull().wp_received
            print(f"Received waypoint {wp_count}. Waypoints pulled")
        except rospy.ServiceException as e:
            print("Service call failed: %s", e)
            return 0

        return 1


def main():

    # FTW = FlyToWP()
    FTM = FlyToMinion()  ## 8====================D

    while not rospy.is_shutdown():
        # if FTW.flyToWp_msg.data and FTW.new_wp_received:
            # if not FTW.wp_pushed:  
            #     print(f"Fly to waypoint.\nPushing wp:\n1. Lat - {FTW.search_wps[0]}\n2. Lon - {FTW.search_wps[1]}\n3. Alt - 8")  
            #     FTW.wp_pushed = FTW.push_wp(FTW.search_wps[0],FTW.search_wps[1],8) 
            
        if FTM.flyToMinion_msg.data and FTM.minion_wp_received:  ## 8====================D
            if not FTM.wp_pushed:  ## 8====================D
                print(f"Fly to Minion.\nPushing wp:\n1. Lat - {FTM.search_wps[0]}\n2. Lon - {FTM.search_wps[1]}\n3. Alt - 8")  ## 8====================D
                FTM.wp_pushed = FTM.push_wp(FTM.search_wps[0],FTM.search_wps[1],8)  ## 8====================D

            # if FTW.state_updated and FTW.uav_state_msg.armed:  
            #     mode = FTW.set_mode(custom_mode='AUTO.MISSION') 
            #     if mode.mode_sent:
            #         print("Mode changed to mission. Executing the current mission.")

            if FTM.state_updated and FTM.uav_state_msg.armed:  ## 8====================D
                mode = FTM.set_mode(custom_mode='AUTO.MISSION')  ## 8====================D
                if mode.mode_sent:
                    print("Mode changed to mission. Executing the current mission.")

                    # FTW.new_wp_received = False
                    FTM.minion_wp_received = False  ## 8====================D

                    # FTW.wp_pushed = False
                    FTM.wp_pushed = False  ## 8====================D

        if FTM.state_updated:  ## 8====================D
            if FTM.uav_state_msg.mode == "AUTO.LOITER":  ## 8====================D
                FTM.wp_reached.data = True  ## 8====================D
                print("waypoint reached.")
            else:
                FTM.wp_reached.data = False  ## 8====================D

        FTM.wp_status_pub.publish(FTM.wp_reached)  ## 8====================D

        FTM.state_updated = False  ## 8====================D

        FTM.rate.sleep()  ## 8====================D


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

