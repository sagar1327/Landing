#!/usr/bin/env python3

import rospy
import math
import numpy as np
from pyproj import Proj, transform
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix
from offboard_py.msg import SearchNReport, Center

class UTMToLLA():
    """Convert UTM coordinates to LLA coordinates (WGS84)."""
    def __init__(self):
        rospy.init_node("rviz_to_lla",anonymous=True)

        self.utm_coordinate_msg = PointStamped()
        rospy.Subscriber("/kevin/initial_point", PointStamped, callback=self.utm_coordinate)

        ## VRX
        # self.boat_coordinate_msg = NavSatFix()
        # self.boat_coordinate_received = False
        # rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, callback=self.boat_coordinate)

        ## Working with minion
        ## TODO: Confirm the pinpoint topic
        self.boat_coordinate_msg = NavSatFix()
        self.boat_coordinate_received = False
        rospy.Subscriber("/minion/sensor/pinpoint/odom", NavSatFix, callback=self.boat_coordinate)

        self.utm_coordinate_msg = PointStamped()
        self.lla_coordinate_msg = SearchNReport()
        self.lla_coordinate_msg.header.frame_id = "map"

        self.lla_coordinate_pub = rospy.Publisher("/kevin/search_report/wp", SearchNReport, queue_size=1)

        self.previous_coordinate = [0.0, 0.0]
        self.coordinate = Center()

        self.rate = rospy.Rate(60)

    def utm_coordinate(self, msg):
        self.utm_coordinate_msg = msg

    def boat_coordinate(self, msg):
        self.boat_coordinate_msg = msg
        self.boat_coordinate_received = True


def main():
    UTL = UTMToLLA()

    while not rospy.is_shutdown():

        if UTL.boat_coordinate_received and (UTL.utm_coordinate_msg.point.x != 0.0 or UTL.utm_coordinate_msg.point.y != 0.0):
            start_lat_in_rad = math.radians(UTL.boat_coordinate_msg.latitude)
            meters_per_degree_lat = 111320
            delta_lat = UTL.utm_coordinate_msg.point.y / meters_per_degree_lat
            meters_per_degree_lon = 111320 * math.cos(start_lat_in_rad)
            delta_long = UTL.utm_coordinate_msg.point.x / meters_per_degree_lon

            new_lat = UTL.boat_coordinate_msg.latitude + delta_lat
            new_long = UTL.boat_coordinate_msg.longitude + delta_long

            if np.abs(new_lat - UTL.previous_coordinate[0]) > 0.00005 or \
                np.abs(new_long - UTL.previous_coordinate[1]) > 0.00005:
                UTL.coordinate.x = new_long
                UTL.coordinate.y = new_lat
                UTL.lla_coordinate_msg.waypoints.append(UTL.coordinate)
            
            UTL.previous_coordinate = [new_lat, new_long]

            UTL.lla_coordinate_msg.header.stamp = rospy.Time.now()
            UTL.lla_coordinate_pub.publish(UTL.lla_coordinate_msg)

        ## Solo testing
        # if UTL.uav_coordinate_received and UTL.sq_center_received and UTL.sq_center_msg.x!=0.0 and UTL.sq_center_msg.y!=0.0:
        #     start_lat_in_rad = math.radians(UTL.uav_coordinate_msg.latitude)
        #     meters_per_degree_lat = 111320
        #     delta_pixel_x = 640-UTL.sq_center_msg.x
        #     delta_pixel_y = 480-UTL.sq_center_msg.y
        #     apx = 48.70141/640
        #     apy = 48.70141/480
        #     alpha = np.abs(delta_pixel_x)*apx*np.pi/180
        #     beta = np.abs(delta_pixel_y)*apy*np.pi/180
        #     deltax_img = np.sign(delta_pixel_x)*np.tan(alpha)*6
        #     deltay_img = -np.sign(delta_pixel_y)*np.tan(beta)*6
        #     delta_lat = deltay_img / meters_per_degree_lat
        #     meters_per_degree_lon = 111320 * math.cos(start_lat_in_rad)
        #     delta_long = deltax_img / meters_per_degree_lon
        #     # print(f"Center: {640-UTL.sq_center_msg.x}, {(480-UTL.sq_center_msg.y)}")

        #     UTL.lla_coordinate_msg.latitude = UTL.uav_coordinate_msg.latitude + delta_lat
        #     UTL.lla_coordinate_msg.longitude = UTL.uav_coordinate_msg.longitude + delta_long

        #     UTL.lla_coordinate_msg.header.stamp = rospy.Time.now()
        #     UTL.lla_coordinate_pub.publish(UTL.lla_coordinate_msg)

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
