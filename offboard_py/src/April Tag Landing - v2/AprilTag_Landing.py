#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from offboard_py.msg import ArTag
from mavros_msgs.msg import *

class LandOnTag():
    """This node sends the flying permission message to the UAV in condition:
        1) Boat is ready (station keep)
        2) There is no April Tag is the given wp.
        
        Requires: 1) Boat Status 2) Tag information 3) A message whether UAV reached the target waypoint or not.
        Outputs: 1) Permission to Fly, 2) Permission to Land on boat."""
    def __init__(self):
        rospy.init_node("Psuedo_publisher", anonymous=True)
        self.wp_reached = Bool()
        self.flyToWp_msg = Bool()
        self.land_on_boat = Bool()
        self.land_on_boat.data = True

        # A psuedo variable to store boat status. Must be replaced later
        self.boat_status_msg = Bool()
        self.boat_status_msg.data = False

        self.artag_msg = ArTag()
        self.flyToWp_msg.data = False
        self.wp_reached_time = None

        rospy.Subscriber("/kevin/waypoint_reached", Bool, callback=self.waypoint)
        rospy.Subscriber("/kevin/artag/info", ArTag, callback=self.artag)
        rospy.Subscriber('mavros/rc/in', RCIn, callback=self.rc_callback)

        # A psuedo subscriber to get the boat status. Must be changed later.
        rospy.Subscriber("/boat/status", Bool, callback=self.boat_status)

        self.flyToWp_pub = rospy.Publisher("/minion/kevin/fly_to_wp", Bool, queue_size=1)
        self.land_on_boat_pub = rospy.Publisher("/kevin/land_permission", Bool, queue_size=1)
        # High publishing rate is not required since once permission is received,
        # it doesn't need to be updated until UAV reached the next waypoint.
        self.rate = rospy.Rate(5)

    def waypoint(self, msg):
        self.wp_reached = msg
        if self.wp_reached.data and self.wp_reached_time is None:
            self.wp_reached_time = rospy.Time.now().to_sec()

    def artag(self, msg):
        self.artag_msg = msg

    def boat_status(self, msg):
        self.boat_status_msg = msg

    def rc_callback(self,msg):
        # Check if the RC switch is in the "Position control" position (Means manual control).
        if msg.channels[4] < 1300:
            rospy.loginfo_once("\nVehicle is in position mode (RC switch). Shutting down script.")
            self.manual_control = True
            rospy.signal_shutdown("\nManual control activated.")
        


def main():

    LOT = LandOnTag()

    while not rospy.is_shutdown():
        # Conditions:
        # 1) If boat is ready and the UAV hasn't reached the given waypoint (Start of a mission.)
        # 2) The UAV reached the given waypoint but no april tag found. Give permission to fly to next waypoint.
        if (LOT.boat_status_msg.data and not LOT.wp_reached.data): #or \
           #(LOT.wp_reached.data and (rospy.Time.now().to_sec() - LOT.wp_reached_time) > 3 and not LOT.artag_msg.detected and not LOT.land_on_boat.data):
            LOT.flyToWp_msg.data = True
                # if LOT.wp_reached.data and not LOT.artag_msg.detected:
                #     print("No April Tag.")      
        # else:
        #     LOT.flyToWp_msg.data = False

        # if (LOT.wp_reached.data and (rospy.Time.now().to_sec() - LOT.wp_reached_time) > 3 and LOT.artag_msg.detected):
        #     LOT.land_on_boat.data = True

        LOT.flyToWp_pub.publish(LOT.flyToWp_msg)
        # LOT.land_on_boat_pub.publish(LOT.land_on_boat)

        LOT.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass