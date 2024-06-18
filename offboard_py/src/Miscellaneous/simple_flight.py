#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class SimpleFilght():
    """Get the current state, arm, and set offboard mode"""
    def __init__(self):

        # Defining current state
        self.current_state = State()
        # Initializing node
        rospy.init_node('simple_flight_node', anonymous=True)
        # Subscriber for getting the current state
        rospy.Subscriber('mavros/state', State, callback=self.state)
        # Defining publisher for the pose
        self.takeoff_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=100)
        self.rate = rospy.Rate(60) # 60 Hz
        self.pose = PoseStamped()
        self.pose.pose.position.x = 27.892023
        self.pose.pose.position.y = 119.272248
        self.pose.pose.position.z = 4
        # Publishing few data points before arming (Neccessary to guarantee arming)
        for i in range(20):
            if rospy.is_shutdown():
                break

            self.takeoff_pub.publish(self.pose)
            self.rate.sleep()
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

        self.arming()
        self.offboard_mode()

    def state(self, msg):
        """Subscriber callback"""
        self.current_state = msg

    def arming(self):
        """For arming the vehicle"""
        if self.current_state.armed:
            rospy.loginfo("Vehicle armed")
        else:
            self.arm.call(self.arm_request)
            if self.arm.call(self.arm_request).success:
                rospy.loginfo("Vehicle armed.")
            else:
                rospy.loginfo("Arming failed.")

    def offboard_mode(self):
        """For enabling offboard mode"""
        if self.current_state.mode == 'OFFBOARD':
            rospy.loginfo('OFFBOARD enabled.')
        else:
            self.set_mode.call(self.mode_request)
            if self.set_mode.call(self.mode_request).mode_sent:
                rospy.loginfo('OFFBOARD enabled.')
            else:
                rospy.loginfo("Failed to set offboard mode.")

    def takeoff(self):
        """Publishes the final pose"""
        while not rospy.is_shutdown():
            self.takeoff_pub.publish(self.pose)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        sf = SimpleFilght()
        sf.takeoff()
    except rospy.ROSInterruptException:
        pass
