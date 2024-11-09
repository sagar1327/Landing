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
        self.flyToMinion_msg = Bool()
        self.flyToMinion_msg.data = False

        # A psuedo variable to store boat status. Must be replaced later
        self.boat_status_msg = Bool()
        self.boat_status_msg.data = True

        rospy.Subscriber("/kevin/artag/info", ArTag, callback=self.artag)

        # A psuedo subscriber to get the boat status. Must be changed later.
        rospy.Subscriber("/minion/kevin/boat/status", Bool, callback=self.boat_status)

        self.flyToMinion_pub = rospy.Publisher("/minion/kevin/fly_to_minion", Bool, queue_size=1)
        # self.land_on_boat_pub = rospy.Publisher("/kevin/land_permission", Bool, queue_size=1)
        # High publishing rate is not required since once permission is received,
        # it doesn't need to be updated until UAV reached the next waypoint.
        self.rate = rospy.Rate(5)

    def boat_status(self, msg):
        self.boat_status_msg = msg
        


def main():

    LOT = LandOnTag()

    while not rospy.is_shutdown():
        # Conditions:
        # 1) If boat is ready and the UAV hasn't reached the given waypoint (Start of a mission.)
        # 2) The UAV reached the given waypoint but no april tag found. Give permission to fly to next waypoint.
        if (LOT.boat_status_msg.data): #and not LOT.wp_reached.data):
            LOT.flyToMinion_msg.data = True
            rospy.loginfo("FLying permission given")
            # if LOT.wp_reached.data and not LOT.artag_msg.detected:
            #     print("No April Tag.")      
        else:
            LOT.flyToMinion_msg.data = False
            rospy.loginfo("FLying permission not given.")

        LOT.flyToMinion_pub.publish(LOT.flyToMinion_msg)

        LOT.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass