#!/usr/bin/env python3

import rospy
import math
from pyproj import Proj, transform
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix

class UTMToLLA():
    """Convert UTM coordinates to LLA coordinates (WGS84)."""
    def __init__(self):
        rospy.init_node("utm_to_lla_conversion",anonymous=True)

        self.utm_coordinate_msg = PointStamped()
        rospy.Subscriber("/kevin/initial_point", PointStamped, callback=self.utm_coordinate)

        self.boat_coordinate_msg = NavSatFix()
        self.boat_coordinate_received = False
        # For onsite testing changing the topic name
        rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, callback=self.boat_coordinate)

        self.lla_coordinate_msg = NavSatFix()
        self.lla_coordinate_msg.header.frame_id = "map"
        self.lla_coordinate_pub = rospy.Publisher("/minion/kevin/target_wp", NavSatFix, queue_size=1)
        self.rate = rospy.Rate(60)

    def utm_coordinate(self, msg):
        self.utm_coordinate_msg = msg
        if msg.point.x != 0.0:
            print("yes")
        print(f"X: {msg.point.x}, Y: {msg.point.y}")

    def boat_coordinate(self, msg):
        self.boat_coordinate_msg = msg
        self.boat_coordinate_received = True


def main():
    UTL = UTMToLLA()

    while not rospy.is_shutdown():
        if UTL.boat_coordinate_received and UTL.utm_coordinate_msg.point.x != 0.0:
            start_lat_in_rad = math.radians(UTL.boat_coordinate_msg.latitude)
            meters_per_degree_lat = 111320
            delta_lat = UTL.utm_coordinate_msg.point.y / meters_per_degree_lat
            meters_per_degree_lon = 111320 * math.cos(start_lat_in_rad)
            delta_long = UTL.utm_coordinate_msg.point.x / meters_per_degree_lon

            UTL.lla_coordinate_msg.latitude = UTL.boat_coordinate_msg.latitude + delta_lat
            UTL.lla_coordinate_msg.longitude = UTL.boat_coordinate_msg.longitude + delta_long

            UTL.lla_coordinate_msg.header.stamp = rospy.Time.now()
            UTL.lla_coordinate_pub.publish(UTL.lla_coordinate_msg)

        UTL.rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
