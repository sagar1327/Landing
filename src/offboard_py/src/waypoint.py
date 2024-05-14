#!/bin/env python3

import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *

class Waypoints():
    def __init__(self):
        rospy.init_node("waypoint_navigation", anonymous=True)

        self.single_wp = Waypoint()
        self.target_wp = []

        rospy.wait_for_service("/mavros/mission/pull")
        self.pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull, persistent=True)

        rospy.wait_for_service("/mavros/mission/push")
        self.push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush, persistent=True)
        
        self.single_wp.frame = 3
        self.single_wp.command = 16
        self.single_wp.autocontinue = True
        self.single_wp.is_current = False
        self.single_wp.param1 = 0.0
        self.single_wp.param2 = 0.0
        self.single_wp.param3 = 0.0
        self.single_wp.param4 = float('nan')
        self.single_wp.x_lat = -33.72097482521049
        self.single_wp.y_long = 150.67109289870723
        self.single_wp.z_alt = 4

        self.target_wp.append(self.single_wp)

        # Push waypoints
        try:
            self.push(start_index=0, waypoints=self.target_wp)
            rospy.loginfo("Waypoint pushed.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        # Pull waypoints
        try:
            wp_count = self.pull().wp_received
            rospy.loginfo("Received %d waypoints. Waypoints pulled.", wp_count)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        Waypoints()
    except rospy.ROSInterruptException:
        pass
