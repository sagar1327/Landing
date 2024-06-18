#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, SetModeRequest

class FailSafe():
    """Obsereve the mode of the drone and trigger failsafe mode."""
    def __init__(self):
        # Defining current state
        self.current_state = State()
        # Setting the failsafe mode
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.mode_request = SetModeRequest()
        self.mode_request.custom_mode = 'AUTO.RTL'
        # Initializing node
        rospy.init_node('failsafe_node', anonymous=True)
        # while not rospy.is_shutdown():
        # Subscriber for getting the current state
        rospy.Subscriber('mavros/state', State, callback=self.failsafe)
        rospy.spin()

    def failsafe(self, msg):
        """Subscriber callback"""
        self.current_state = msg
        if self.current_state.mode == 'POSCTL':
            self.set_mode.call(self.mode_request)
            if self.set_mode.call(self.mode_request).mode_sent:
                rospy.loginfo('FAILSAFE Mode enabled.')
                rospy.signal_shutdown('Failsafe enabling complete.')
        else:
            rospy.loginfo('Waiting...')


if __name__ == '__main__':
    try:
        fs = FailSafe()
        
    except rospy.ROSInterruptException:
        pass