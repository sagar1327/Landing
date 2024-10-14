#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from offboard_py.msg import ArTag

class PsuedoPublisher():
    """This node sends the flying permission message to the UAV in condition:
        1) Boat is ready (station keep)
        2) There is no April Tag is the given wp."""
    def __init__(self):
        rospy.init_node("Psuedo_publisher", anonymous=True)
        self.wp_reached = Bool()
        self.flyToWp_msg = Bool()

        # A psuedo variable to store boat status. Must be replaced later
        self.boat_status_msg = String()
        self.boat_status_msg.data = "Ready"

        self.artag_msg = ArTag()
        self.flyToWp_msg.data = False
        self.wp_reached_time = None

        rospy.Subscriber("/kevin/waypoint_reached", Bool, callback=self.waypoint)
        rospy.Subscriber("/kevin/artag/info", ArTag, callback=self.artag)

        # A psuedo subscriber to get the boat status. Must be changed later.
        rospy.Subscriber("/boat/status", String, callback=self.boat_status)

        self.flyToWp_pub = rospy.Publisher("/minion/kevin/fly_to_wp", Bool, queue_size=1)
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
        


def main():

    PP = PsuedoPublisher()

    while not rospy.is_shutdown():
        # Conditions:
        # 1) If boat is ready and the UAV hasn't reached the given waypoint (Start of a mission.)
        # 2) The UAV reached the given waypoint but no april tag found. Give permission to fly to next waypoint.
        if (PP.boat_status_msg.data == "Ready" and not PP.wp_reached.data) or \
           (PP.wp_reached.data and (rospy.Time.now().to_sec() - PP.wp_reached_time) > 3 and not PP.artag_msg.detected):
                PP.flyToWp_msg.data = True
                if PP.wp_reached.data and not PP.artag_msg.detected:
                    print("No April Tag.")      
        else:
            PP.flyToWp_msg.data = False

        PP.flyToWp_pub.publish(PP.flyToWp_msg)

        PP.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass