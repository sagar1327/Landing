#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, WaypointPull, SetMode
from offboard_py.msg import MissionStatus, ArTag


class FlyToWP():
    """A node to make the drone takeoff and fly to minion.
       Requires: 1) Minion GPS Coordinates. 2) Permission to takeoff."""
    def __init__(self):

        rospy.init_node("Fly_to_minion",anonymous=True)
        self.uav_state_msg = State()
        self.single_wp = Waypoint()
        # self.flyToMinion_msg = Bool()
        # self.flyToMinion_msg.data = False
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

        # self.search_wps = [0,0]
        self.state_updated = False

        self.boat_status_msg = Bool()

        self.usv_coordinate_msg = NavSatFix()
        self.usv_coordinate_received = False

        self.takeoff_time = rospy.Time.now().to_sec()
        
        rospy.wait_for_service("/mavros/mission/pull")
        self.pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull, persistent=True)
        rospy.wait_for_service("/mavros/mission/push")
        self.push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush, persistent=True)
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # rospy.Subscriber("/kevin/search_report/wp", NavSatFix, callback=self.waypoint)
        rospy.Subscriber('mavros/state', State, callback=self.uav_state)
        # rospy.Subscriber("/minion/kevin/fly_to_minion", Bool, callback=self.flyToMinion)
        # rospy.Subscriber("/kevin/square_target", SquareTarget, callback=self.sq_target)
        rospy.Subscriber("/minion/kevin/boat/status", Int64, callback=self.boat_status)
        rospy.Subscriber("/minion/sensors/pinpoint/fix", NavSatFix, callback=self.usv_coordinate)

        self.wp_status_pub = rospy.Publisher("/kevin/waypoint_reached", Bool, queue_size=1)
        self.rate = rospy.Rate(5)

    def uav_state(self, msg):
        self.uav_state_msg = msg
        self.state_updated = True

    def usv_coordinate(self, msg):
        if np.abs(msg.latitude - self.search_wps[0]) > 0.0001 or np.abs(msg.longitude - self.search_wps[1]) > 0.0001:
            self.search_wps = [msg.latitude, msg.longitude]
            self.usv_coordinate_received = True

    # def flyToMinion(self, msg):
    #     self.flyToMinion_msg = msg

    def boat_status(self, msg):
        self.boat_status_msg = msg

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

    FTW = FlyToWP()

    while not rospy.is_shutdown():
        # print(FTW.flyToMinion_msg.data)
        if FTW.boat_status_msg.data == 1 and FTW.state_updated and FTW.uav_state_msg.armed 
           and not FTW.uav_state_msg.mode == "AUTO.TAKEOFF":
            mode = FTW.set_mode(custom_mode='AUTO.TAKEOFF')
                if mode.mode_sent:
                    print("Mode changed to AUTO.TAKEOFF. TakingOff ...")
            
            time_threshold = 10
        else:
            print(f"Current mode is {FTW.uav_state_msg.mode}. Kevin is in the air.")
            time_threshold = 3
        
        if FTW.boat_status_msg.data == 2 and (rospy.Time.now().to_sec() -  FTW.takeoff_time) > time_threshold and
           and FTW.state_updated and FTW.uav_state_msg.mode == "AUTO.LOITER":
            if not FTW.wp_pushed:
                print(f"Fly to wp.\nPushing wp:\n1. Lat - {FTW.search_wps[0]}\n2. Lon - {FTW.search_wps[1]}\n3. Alt - 5")
                FTW.wp_pushed = FTW.push_wp(FTW.search_wps[0],FTW.search_wps[1],4)
                mode = FTW.set_mode(custom_mode='AUTO.MISSION')
                if mode.mode_sent:
                    print("Mode changed to mission. Flying towards minion.")

        # if FTW.state_updated:
        #     if FTW.uav_state_msg.mode == "AUTO.LOITER":
        #         FTW.wp_reached.data = True
        #         FTW.target_reached_time = rospy.Time.now().to_sec()
        #         print("waypoint reached.")
        #     else:
        #         FTW.wp_reached.data = False

        # if FTW.wp_reached.data and (rospy.Time.now().to_sec() - FTW.target_reached_time) > 3 and not FTW.square_target_msg.target[0].detected:
        #     FTW.go_new_wp = True

        # FTW.wp_status_pub.publish(FTW.wp_reached)

        FTW.state_updated = False

        FTW.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

