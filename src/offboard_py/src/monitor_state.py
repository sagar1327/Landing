#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class MonitorState():
    """Get the current state, arm, and set offboard mode"""
    def __init__(self):

        # Initializing node
        rospy.init_node('monitor_state_node', anonymous=True)

        # Defining current state
        self.current_state = State()
        self.initialize_time = rospy.Time.now().to_sec()
        self.arm_time = None
        self.takeoff_alt_reached = False

        # Subscriber for getting the current state
        rospy.Subscriber('mavros/state', State, callback=self.monitor_state)
        # Subscriber for current position
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.current_position)

        # Setiing the Bool value for arming
        # rospy.wait_for_service("/mavros/cmd/arming")
        # self.arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        # self.arm_request = CommandBoolRequest()
        # self.arm_request.value = True
        
        # Setting the custom mode
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.spin()
    
    def current_position(self,msg):
        """Callback function for the current position subscriber updating vehicle position"""
        self.current_pose = msg

    def monitor_state(self, msg):
        """Monitor and change curent state."""
        self.current_state = msg
        rospy.loginfo("Arming status: " + str(self.current_state.armed) + ", " + "Mode: " + self.current_state.mode)
        
        # Check and wait for arming
        if rospy.Time.now().to_sec() - self.initialize_time > 3 and not self.current_state.armed:
            rospy.loginfo_once("Vehicle is ready to arm...")
        elif self.current_state.armed and self.arm_time is None:
            rospy.loginfo_once("Vehicle armed.")
            self.arm_time = rospy.Time.now().to_sec()

        # If armed, takeoff to default altitude (2.5 m)
        if self.current_state.armed and rospy.Time.now().to_sec() - self.arm_time > 3 and not self.takeoff_alt_reached:
            if self.current_state.mode != "AUTO.TAKEOFF":
                mode = self.set_mode(custom_mode='AUTO.TAKEOFF')
                if mode.mode_sent:
                    rospy.loginfo_once("Vehicle is taking off.")

            if self.current_state.mode == "AUTO.LOITER":
                self.takeoff_alt_reached = True
                mode = self.set_mode(custom_mode='OFFBOARD')
                if mode.mode_sent:
                    rospy.loginfo_once("Vehicle is in OFFBOARD mode.")

        if self.current_state.mode == 'POSCTL':
            mode = self.set_mode(custom_mode='AUTO.RTL')
            if mode.mode_sent:
                rospy.loginfo('FAILSAFE Mode enabled.')
                rospy.signal_shutdown('Failsafe enabling complete.')



if __name__ == '__main__':
    try:
        ms = MonitorState()
    except rospy.ROSInterruptException:
        pass
