#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, WaypointPull
from offboard_py.msg import MissionStatus
from std_msgs.msg import Bool

class MonitorState():
    def __init__(self):

        rospy.init_node('monitor_state_node', anonymous=True)

        self.current_state_msg = State()
        self.initialize_time = rospy.Time.now().to_sec()
        self.takeoff_alt_reached = False
        self.landing_condition_msg = Bool()
        self.mission_status_msg = MissionStatus()
        self.wp_received = False

        rospy.Subscriber('mavros/state', State, callback=self.current_state)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.current_position)
        rospy.Subscriber("/kevin/landing", Bool, callback=self.landing_condition)
        rospy.Subscriber("/kevin/mission/status", MissionStatus, callback=self.mission_status)
        self.mission_status_pub = rospy.Publisher("/kevin/mission/status", MissionStatus, queue_size=1)
        self.rate = rospy.Rate(1)

        rospy.wait_for_service("/mavros/mission/pull")
        self.pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull, persistent=True)
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    
    def current_position(self,msg):
        self.current_pose = msg

    def landing_condition(self, msg):
        self.landing_condition_msg = msg

    def current_state(self, msg):
        self.current_state_msg = msg
        print("\nArming status: {}, Mode: {}".format(self.current_state_msg.armed,self.current_state_msg.mode))

    def mission_status(self, msg):
        self.mission_status_msg = msg

    def pull_wp(self):
        try:
            wp_count = self.pull().wp_received
            if wp_count > 0:
                # print(f"\nReceived waypoint {wp_count}. Waypoints pulled.\n")
                return 1
            else:
                return 0
        except rospy.ServiceException as e:
            print("\nService call failed: %s", e)
            return 0


def main():

    MS = MonitorState()
    while not rospy.is_shutdown():

        # Change to landing mode.
        if MS.landing_condition_msg.data:
            mode = MS.set_mode(custom_mode="AUTO.LAND")
            if mode.mode_sent:
                print("\nUAV now landing.")

        # Check and wait for arming
        if rospy.Time.now().to_sec() - MS.initialize_time > 1: 
            
            # Check for new mission
            if MS.mission_status_msg.new_mission_request and not MS.mission_status_msg.new_mission_pulled:
                print("\nWaiting for waypoints")
                MS.wp_received = MS.pull_wp()
            
            if MS.wp_received and MS.current_state_msg.mode != 'AUTO.MISSION':
                MS.mission_status_msg.header.stamp = rospy.Time.now()
                MS.mission_status_msg.header.frame_id = 'map'
                MS.mission_status_msg.new_mission_request = True
                MS.mission_status_msg.new_mission_pushed = True
                MS.mission_status_msg.new_mission_pulled = True

                MS.mission_status_pub.publish(MS.mission_status_msg)

                if not MS.current_state_msg.armed:
                    pass
                    # print("\nVehicle is ready to arm.")
                elif MS.current_state_msg.armed:
                    # print("\nVehicle armed.")
                    # print(MS.current_state_msg.mode)
                    mode = MS.set_mode(custom_mode='AUTO.MISSION')
                    if mode.mode_sent:
                        print("\nMode changed to mission. Executing the current mission.\n")

                    MS.wp_received = False

        MS.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
