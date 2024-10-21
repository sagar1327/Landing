#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix


class TestPublisher():
    """(Only for testing) A psuedo publisher to publish custom waypoints for gazebo and testing purposes.
       Real Time: Ask Boat to directly publish target waypoint in /minion/kevin/target_wp topic. In that case, no need to use this node."""
    def __init__(self):

        rospy.init_node("test_publisher")
        
        self.wamv_coordinate_msg = NavSatFix()
        self.coordinate_received = False
        rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, callback=self.wamv_coordinate)

        self.target_wp_msg = NavSatFix()
        # For when manually giving the waypoints.
        # self.target_wp_msg.latitude = 29.1826494
        # self.target_wp_msg.longitude = -81.0441421

        self.target_wp_msg.header.frame_id = "map"
        self.target_wp_pub = rospy.Publisher("/minion/kevin/target_wp", NavSatFix, queue_size=3)


        self.rate = rospy.Rate(60)

    def wamv_coordinate(self, msg):
        self.wamv_coordinate_msg = msg
        self.coordinate_received = True


def main():

    TP = TestPublisher()

    while not rospy.is_shutdown():
        if TP.coordinate_received:
            TP.target_wp_msg = TP.wamv_coordinate_msg
            # Change the lat and long recieved from boat if required.
            TP.target_wp_msg.longitude = TP.target_wp_msg.longitude - 0.00006

        TP.target_wp_pub.publish(TP.target_wp_msg)

        TP.coordinate_received = False
        TP.rate.sleep()


if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass
