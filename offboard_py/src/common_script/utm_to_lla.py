#!/usr/bin/env python3

import rospy
import math
from pyproj import Proj, transform
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix
from offboard_py.msg import Center

class UTMToLLA():
    """Convert UTM coordinates to LLA coordinates (WGS84)."""
    def __init__(self):
        rospy.init_node("utm_to_lla_conversion",anonymous=True)

        # ## Use it when working with minion/vrx
        # self.utm_coordinate_msg = PointStamped()
        # rospy.Subscriber("/kevin/initial_point", PointStamped, callback=self.utm_coordinate)

        ## VRX use only
        # self.boat_coordinate_msg = NavSatFix()
        # self.boat_coordinate_received = False
        # rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, callback=self.boat_coordinate)

        ## Working with minion
        # self.boat_coordinate_msg = NavSatFix()
        # self.boat_coordinate_received = False
        # rospy.Subscriber("/minion/pinpoint/odom", NavSatFix, callback=self.boat_coordinate)

        ## Solo testing the drone
        self.sq_center_msg = Center()
        self.uav_coordinate_msg = NavSatFix()
        self.utm_coordinate_msg = PointStamped()
        self.lla_coordinate_msg = NavSatFix()
        self.lla_coordinate_msg.header.frame_id = "map"

        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, callback=self.uav_coordinate)
        rospy.Subscriber("/kevin/target/squares/center", Center, callback=self.sq_center)
        self.lla_coordinate_pub = rospy.Publisher("/kevin/search_report/inidividual/target/wp", NavSatFix, queue_size=1)

        # self.utm_coordinate_msg.point.x = -6.0 # Change value as required
        # self.utm_coordinate_msg.point.y = 0.0 # Change value as required

        self.uav_coordinate_received = False
        self.sq_center_received = False

        self.rate = rospy.Rate(60)

    # def utm_coordinate(self, msg):
    #     self.utm_coordinate_msg = msg
    #     if msg.point.x != 0.0:
    #         print("yes")
    #     print(f"X: {msg.point.x}, Y: {msg.point.y}")

    # def boat_coordinate(self, msg):
    #     self.boat_coordinate_msg = msg
    #     self.boat_coordinate_received = True

    def sq_center(self,msg):
        self.sq_center_msg = msg
        self.sq_center_received = True

    def uav_coordinate(self, msg):
        self.uav_coordinate_msg = msg
        self.uav_coordinate_received = True


def main():
    UTL = UTMToLLA()

    while not rospy.is_shutdown():

        # ## VRX or Minion
        # if UTL.boat_coordinate_received and UTL.utm_coordinate_msg.point.x != 0.0:
        #     start_lat_in_rad = math.radians(UTL.boat_coordinate_msg.latitude)
        #     meters_per_degree_lat = 111320
        #     delta_lat = UTL.utm_coordinate_msg.point.y / meters_per_degree_lat
        #     meters_per_degree_lon = 111320 * math.cos(start_lat_in_rad)
        #     delta_long = UTL.utm_coordinate_msg.point.x / meters_per_degree_lon

        #     UTL.lla_coordinate_msg.latitude = UTL.boat_coordinate_msg.latitude + delta_lat
        #     UTL.lla_coordinate_msg.longitude = UTL.boat_coordinate_msg.longitude + delta_long

        #     UTL.lla_coordinate_msg.header.stamp = rospy.Time.now()
        #     UTL.lla_coordinate_pub.publish(UTL.lla_coordinate_msg)

        ## Solo testing
        if UTL.uav_coordinate_received and UTL.sq_center_received and UTL.sq_center_msg.x!=0.0 and UTL.sq_center_msg.x!=0.0:
            start_lat_in_rad = math.radians(UTL.uav_coordinate_msg.latitude)
            meters_per_degree_lat = 111320
            delta_lat = UTL.sq_center_msg.y / meters_per_degree_lat
            meters_per_degree_lon = 111320 * math.cos(start_lat_in_rad)
            delta_long = UTL.sq_center_msg.x / meters_per_degree_lon

            UTL.lla_coordinate_msg.latitude = UTL.uav_coordinate_msg.latitude + delta_lat
            UTL.lla_coordinate_msg.longitude = UTL.uav_coordinate_msg.longitude + delta_long

            UTL.lla_coordinate_msg.header.stamp = rospy.Time.now()
            UTL.lla_coordinate_pub.publish(UTL.lla_coordinate_msg)

        UTL.rate.sleep()

####### TODO #########
# Get the waypoint of boat/drone
# Store it in variable so that it doesn't change everyloop
# Use that to calculate the new wp,


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
