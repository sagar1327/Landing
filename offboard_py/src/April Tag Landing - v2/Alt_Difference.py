#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from offboard_py.msg import ArTagAltitude


class DeltaAlt():
    """Find the difference between the current z-position given by GPS module and estimated height using April Tag."""
    def __init__(self):

        rospy.init_node("uav_control_node",anonymous=True)
        self.uav_pose_msg = PoseStamped()
        self.artag_altitude_msg = ArTagAltitude()
        self.initial_alt = None
        self.alt_from_tag_received = False

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.uav_pose)
        rospy.Subscriber("/kevin/artag/altitude", ArTagAltitude, callback=self.artag_altitude)

        rospy.spin()

    def uav_pose(self, msg):
        self.uav_pose_msg = msg
        if self.initial_alt is None:
            self.initial_alt = self.uav_pose_msg.pose.position.z
        actual_height = self.uav_pose_msg.pose.position.z - self.initial_alt

        if self.alt_from_tag_received and self.artag_altitude_msg.altitude != np.Inf:
            print(f"GPS height: {actual_height}.")
            print(f"Estimated height: {self.artag_altitude_msg.altitude}.")
            print(f"Current difference in GPS and tag alt: {actual_height - self.artag_altitude_msg.altitude}")

            self.alt_from_tag_received = False

    def artag_altitude(self, msg):
        self.artag_altitude_msg = msg
        self.alt_from_tag_received = True


if __name__ == '__main__':
    try:
        Ct = DeltaAlt()
    except rospy.ROSInterruptException:
        pass