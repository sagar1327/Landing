#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from offboard_py.msg import Center,SquareCenters
from mavros_msgs.srv import SetMode


class SearchNReport():

    def __init__(self):
        rospy.init_node("search_report")

        self.uav_state_msg = State()
        self.sq_cent_msg = SquareCenters()
        self.centers = []
        self.all_centers_received = False

        rospy.Subscriber("/mavros/state",State,callback=self.uav_state)
        rospy.Subscriber("/kevin/target/squares/center",State,callback=self.square_center)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def uav_state(self,msg):
        self.uav_state_msg = msg

def main():
    SNR = SearchNReport()

    if SNR.uav_state_msg.mode == "AUTO.MISSION" and len(SNR.sq_cent_msg.centers)!=0:
        mode = SNR.set_mode(custom_mode='AUTO.LOITER')
        if mode.mode_sent:
            print("Mode changed to mission. Executing the current mission.")

            SNR.new_wp_received = False
            SNR.wp_pushed = False


