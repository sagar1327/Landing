#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush
from offboard_py.msg import MissionStatus
from std_msgs.msg import String


class PushWaypoints():

    def __init__(self):

        rospy.init_node("publish_waypoints", anonymous=True)

        self.single_wp = Waypoint()
        self.single_wp.frame = 3
        self.single_wp.command = 16
        self.single_wp.autocontinue = True
        self.single_wp.is_current = False
        self.single_wp.param1 = 0.0
        self.single_wp.param2 = 0.0
        self.single_wp.param3 = 0.0
        self.single_wp.param4 = float('nan')
        self.target_wp = []
        self.wp_pushed = False

        self.wamv_coordinate_msg = NavSatFix()
        self.mission_status_msg = MissionStatus()

        # ROS service
        rospy.wait_for_service("/mavros/mission/push")
        self.push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush, persistent=True)

        rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, callback=self.wamv_location)
        rospy.Subscriber("/kevin/mission/status", MissionStatus, callback=self.mission_status)
        self.mission_status_pub = rospy.Publisher("/kevin/mission/status", MissionStatus, queue_size=1)
        self.rate = rospy.Rate(60)

    def wamv_location(self, msg):
        self.wamv_coordinate_msg = msg
        # self.wamv_coordinate_msg_received = True

    def mission_status(self, msg):
        self.mission_status_msg = msg

    def push_wp(self,lat,lon,alt):

        self.single_wp.x_lat =  lat
        self.single_wp.y_long = lon
        self.single_wp.z_alt = alt

        self.target_wp = [self.single_wp]

        # Push waypoints.
        try:
            self.push(start_index=0, waypoints=self.target_wp)
            # rospy.loginfo("\nWaypoint pushed\n.")
        except rospy.ServiceException as e:
            print("Service call failed: %s", e)


def main():

    PW = PushWaypoints()
    while not rospy.is_shutdown():
        if PW.mission_status_msg.new_mission_request and not PW.mission_status_msg.new_mission_pulled:
            if not PW.wp_pushed:
                lat = -33.72096147064639
                lon = 150.67130545444115 - 0.00005
                alt = 5
                PW.push_wp(lat, lon, alt)
                # print("New waypoint pushed.")
                PW.wp_pushed = True

            PW.mission_status_msg.header.stamp = rospy.Time.now()
            PW.mission_status_msg.header.frame_id = 'map'
            PW.mission_status_msg.new_mission_request = True
            PW.mission_status_msg.new_mission_pushed = True
            PW.mission_status_msg.new_mission_pulled = False

            PW.mission_status_pub.publish(PW.mission_status_msg)
        else:
            PW.wp_pushed = False
            # print("Waypoint pulled.")

        PW.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass