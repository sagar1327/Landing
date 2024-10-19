#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import RCIn


class ManualTakeover():
    """Stop all autonomous node. Pilot gets full control."""
    def __init__(self):
        rospy.init_node("manual_takeover_node",anonymous=True)
        self.takeover_msg = RCIn()
        rospy.Subscriber('mavros/rc/in', RCIn, callback=self.rc_callback)

        rospy.spin()

    def rc_callback(self, msg):
        self.takeover_msg = msg
        # Check if the RC switch is in the "Position control" position (Means manual control).
        if self.takeover_msg.channels[4] < 1300:
            rospy.loginfo_once("\nVehicle is in position mode (RC switch). Shutting down script.")
            rospy.signal_shutdown("Manual control activated.")


def main():
    MT = ManualTakeover()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass