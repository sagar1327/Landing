#!/usr/bin/env python3

import rospy
import numpy as np
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, WaypointPull, SetMode
from offboard_py.msg import MissionStatus, ArTag


class FlyToWP():
    """A node to make the drone fly to a waypoint."""
    def __init__(self):

        rospy.init_node("Fly_to_wp",anonymous=True)

        self.wamv_coordinate_msg = NavSatFix()
        self.uav_state_msg = State()
        self.single_wp = Waypoint()
        self.mission_status_msg = MissionStatus()
        self.artag_msg = ArTag()

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

        self.mission_status_msg.new_mission_request = True

        self.search_wps = []
        self.target_wp_received = False
        self.wamv_coordinate_received = False
        self.mode_changed_to_mission = False
        self.state_updated = False
        self.targetWP_reached = False
        self.targetWP_reached_time = None
        
        rospy.wait_for_service("/mavros/mission/pull")
        self.pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull, persistent=True)
        rospy.wait_for_service("/mavros/mission/push")
        self.push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush, persistent=True)
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, callback=self.wamv_coordinate)
        rospy.Subscriber('mavros/state', State, callback=self.uav_state)
        rospy.Subscriber("/kevin/mission/status", MissionStatus, callback=self.mission_status)
        rospy.Subscriber("/kevin/artag/info", ArTag, callback=self.artag)

        self.search_limit = rospy.get_param('search_limit', [0, 0])

        self.mission_status_pub = rospy.Publisher("/kevin/mission/status", MissionStatus, queue_size=1)
        self.rate = rospy.Rate(60)

    def wamv_coordinate(self, msg):
        self.wamv_coordinate_msg = msg

        self.search_wps = [[msg.latitude,msg.longitude],
                           [msg.latitude,msg.longitude - self.search_limit[1]],
                           [msg.latitude - self.search_limit[0],msg.longitude - self.search_limit[1]],
                            [msg.latitude + self.search_limit[0],msg.longitude - self.search_limit[1]]]
        # self.search_wps = [[msg.latitude,msg.longitude - 0.00005]]
        
        self.wamv_coordinate_received = True

    def uav_state(self, msg):
        self.uav_state_msg = msg
        self.state_updated = True
        # print(f"State update: {self.state_updated}")

    def mission_status(self, msg):
        self.mission_status_msg = msg

    def artag(self, msg):
        self.artag_msg = msg

    def push_wp(self,lat,lon,alt):
        self.single_wp.x_lat =  lat
        self.single_wp.y_long = lon
        self.single_wp.z_alt = alt

        target_wp = [self.single_wp]

        if self.mission_status_msg.new_mission_request:
            # Push waypoints.
            try:
                self.push(start_index=0, waypoints=target_wp)
                print("Waypoint pushed.")
            except rospy.ServiceException as e:
                print("Service call failed: %s", e)
                return 0

            # Pull waypoints.
            try:
                wp_count = self.pull().wp_received
                print(f"Received waypoint {wp_count}. Waypoints pulled")
            except rospy.ServiceException as e:
                print("Service call failed: %s", e)
                return 0

            return 1


def main():

    FTW = FlyToWP()

    while not rospy.is_shutdown():
        
        if not FTW.mission_status_msg.landing:
            # print(f"Mission request: {FTW.mission_status_msg.new_mission_request}\nMission pushed: {FTW.mission_status_msg.new_mission_pushed}\n"+
            #       f"Mission pulled: {FTW.mission_status_msg.new_mission_pulled}\nMission active: {FTW.mission_status_msg.mission_active}\n"+
            #       f"Mission complete: {FTW.mission_status_msg.mission_complete}")
            # print(f"Wamv coordinate received: {FTW.wamv_coordinate_received}")
            # print(f"State update: {FTW.state_updated}")
            # print(f"Mode: {FTW.uav_state_msg.mode}")
            # Push and Pull initial waypoint of the boat.
            if FTW.mission_status_msg.new_mission_request and FTW.wamv_coordinate_received:
                if not FTW.mission_status_msg.new_mission_pushed and not FTW.mission_status_msg.new_mission_pulled:
                    wp_idx = np.random.randint(0,4)
                    print(f"Wp idx: {wp_idx}")
                    print(f"New mission Request.\nPushing wp:\n1. Lat - {FTW.search_wps[wp_idx][0]}\n2. Lon - {FTW.search_wps[wp_idx][1]}\n3. Alt - 6")
                    # print(rospy.Time.now().to_sec())
                    ret = FTW.push_wp(FTW.search_wps[wp_idx][0],FTW.search_wps[wp_idx][1],6)
                    if ret:            
                        FTW.mission_status_msg.new_mission_pushed = True
                        FTW.mission_status_msg.new_mission_pulled = True

            # Switch to mission mode
            # print(f"Mission request: {FTW.mission_status_msg.new_mission_request}\nMission pushed: {FTW.mission_status_msg.new_mission_pushed}\n"+
            #       f"Mission pulled: {FTW.mission_status_msg.new_mission_pulled}\nMission active: {FTW.mission_status_msg.mission_active}\n"+
            #       f"Mission complete: {FTW.mission_status_msg.mission_complete}")
            # print(f"State update: {FTW.state_updated}")
            # print(f"Mode: {FTW.uav_state_msg.mode}")
            if FTW.state_updated and FTW.uav_state_msg.armed and FTW.mission_status_msg.new_mission_pushed and not FTW.mission_status_msg.mission_active:
                mode = FTW.set_mode(custom_mode='AUTO.MISSION')
                if mode.mode_sent:
                    print("Mode changed to mission. Executing the current mission.")
                    # print(rospy.Time.now().to_sec())
                FTW.mission_status_msg.mission_active = True
                FTW.mission_status_msg.new_mission_request = False
                FTW.mission_status_msg.new_mission_pushed = False
                FTW.mission_status_msg.new_mission_pulled = False

                FTW.state_updated = False

            # If uav reach the target waypoint.
            # print(f"Mission request: {FTW.mission_status_msg.new_mission_request}\nMission pushed: {FTW.mission_status_msg.new_mission_pushed}\n"+
            #       f"Mission pulled: {FTW.mission_status_msg.new_mission_pulled}\nMission active: {FTW.mission_status_msg.mission_active}\n"+
            #       f"Mission complete: {FTW.mission_status_msg.mission_complete}")
            # print(f"State update: {FTW.state_updated}")
            # print(f"Mode: {FTW.uav_state_msg.mode}")
            if FTW.state_updated and FTW.uav_state_msg.armed and FTW.uav_state_msg.mode == "AUTO.LOITER" and \
            FTW.mission_status_msg.mission_active and not FTW.mission_status_msg.mission_complete:
                print("Target wp reached.")
                # print(rospy.Time.now().to_sec())
                FTW.targetWP_reached_time = rospy.Time.now().to_sec()
                FTW.mission_status_msg.mission_complete = True
                FTW.mission_status_msg.mission_active = False

            # print(f"Mission request: {FTW.mission_status_msg.new_mission_request}\nMission pushed: {FTW.mission_status_msg.new_mission_pushed}\n"+
            #       f"Mission pulled: {FTW.mission_status_msg.new_mission_pulled}\nMission active: {FTW.mission_status_msg.mission_active}\n"+
            #       f"Mission complete: {FTW.mission_status_msg.mission_complete}")
            # print(f"State update: {FTW.state_updated}")
            # print(f"Mode: {FTW.uav_state_msg.mode}")
            if FTW.mission_status_msg.mission_complete and (rospy.Time.now().to_sec()-FTW.targetWP_reached_time) > 3:
                # print(f"State update: {FTW.state_updated}")
                # print(f"Mode: {FTW.uav_state_msg.mode}")
                # print(FTW.artag_msg)
                if (not FTW.artag_msg.detected and not FTW.mission_status_msg.landing):
                    print("No Apriltag.")
                    # print(rospy.Time.now().to_sec())
                    FTW.mission_status_msg.new_mission_request = True
                    FTW.mission_status_msg.mission_complete = False
                else:
                    FTW.mission_status_msg.landing = True


        FTW.mission_status_pub.publish(FTW.mission_status_msg)

        FTW.wamv_coordinate_received = False
        FTW.state_updated = False

        FTW.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

