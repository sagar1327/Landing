#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class MonitorState():
    """Get the current state, arm, and set offboard mode"""
    def __init__(self):

        # Defining current state
        self.current_state = State()

        # Initializing node
        rospy.init_node('monitor_state_node', anonymous=True)
        # Subscriber for getting the current state
        rospy.Subscriber('mavros/state', State, callback=self.monitor_state)

        # Setiing the Bool value for arming
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.arm_request = CommandBoolRequest()
        self.arm_request.value = True
        
        # Setting the custom mode
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.mode_request = SetModeRequest()
        self.mode_request.custom_mode = 'OFFBOARD'

        rospy.spin()

    def monitor_state(self, msg):
        """Monitor and change curent state."""
        self.current_state = msg
        rospy.loginfo("Arming status: " + str(self.current_state.armed) + ", " + "Mode: " + self.current_state.mode)
        if not self.current_state.armed:
            self.arm.call(self.arm_request)
            if self.arm.call(self.arm_request).success:
                rospy.loginfo("Vehicle armed. Switching mode to OFFBOARD...")       
            self.set_mode.call(self.mode_request)
            if self.set_mode.call(self.mode_request).mode_sent:
                rospy.loginfo('OFFBOARD mode enabled.')

        if self.current_state.mode == 'POSCTL':
            self.mode_request.custom_mode = 'AUTO.RTL'
            self.set_mode.call(self.mode_request)
            if self.set_mode.call(self.mode_request).mode_sent:
                rospy.loginfo('FAILSAFE Mode enabled.')
                rospy.signal_shutdown('Failsafe enabling complete.')



if __name__ == '__main__':
    try:
        ms = MonitorState()
    except rospy.ROSInterruptException:
        pass
