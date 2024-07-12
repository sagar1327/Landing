#!/usr/bin/env python3

import rospy
from offboard_py.msg import MissionStatus
from mavros_msgs.msg import State

class PsuedoPublisher():

    def __init__(self):

        rospy.init_node("Psuedo_publisher", anonymous=True)
        self.mission_status_msg = MissionStatus()
        self.current_state_msg = State()

        rospy.Subscriber("/mavros/state", State, callback=self.current_state)
        rospy.Subscriber("/kevin/mission/status", MissionStatus, callback=self.mission_status)
        self.mission_status_pub = rospy.Publisher("/kevin/mission/status", MissionStatus, queue_size=1)
        self.rate = rospy.Rate(60)

    def current_state(self, msg):
        self.current_state_msg = msg

    def mission_status(self, msg):
        self.mission_status_msg = msg


def main():

    PP = PsuedoPublisher()

    while not rospy.is_shutdown():

        # print(PP.mission_status_msg.new_mission_pushed)
        if not PP.mission_status_msg.new_mission_pushed:

            PP.mission_status_msg.header.stamp = rospy.Time.now()
            PP.mission_status_msg.header.frame_id = 'map'
            PP.mission_status_msg.new_mission_request = True
            PP.mission_status_msg.new_mission_pushed = False
            PP.mission_status_msg.new_mission_pulled = False

            # print("If statement running.")
            PP.mission_status_pub.publish(PP.mission_status_msg)

        elif PP.current_state_msg.mode == "AUTO.MISSION":

            PP.mission_status_msg.header.stamp = rospy.Time.now()
            PP.mission_status_msg.header.frame_id = 'map'
            PP.mission_status_msg.new_mission_request = False
            PP.mission_status_msg.new_mission_pushed = True
            PP.mission_status_msg.new_mission_pulled = True

            # print("ElIf statement running.")
            PP.mission_status_pub.publish(PP.mission_status_msg)

        # else:
            # print("Else statement running.")

        PP.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass