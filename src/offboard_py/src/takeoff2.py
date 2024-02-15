#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.msg import ModelStates, ModelState
from tf.transformations import euler_from_quaternion

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class TakeOff():
    def __init__(self):
        rospy.init_node('takeoff_node', anonymous=True)
        self.velocity_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=100)
        self.rate = rospy.Rate(30)
        self.current_vel = TwistStamped()
        self.current_pose = ModelState()
        self.wamv_pose = ModelState()

        # Initialize PID controllers
        self.pid_x = PIDController(kp=0.1, ki=0.01, kd=1.5)
        self.pid_y = PIDController(kp=0.02, ki=0.01, kd=1.5)
        self.pid_z = PIDController(kp=1, ki=0.1, kd=0.01)
        self.pid_yaw = PIDController(kp=0.5, ki=0.01, kd=1.5)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

        while not rospy.is_shutdown():
            self.calculate_vel()
            self.velocity_pub.publish(self.current_vel)
            rospy.loginfo("\ndrone velocity:\n{}".format(self.current_vel))
            self.rate.sleep()

    def callback(self, msg):
        if msg.name[22] == "wamv":
            self.current_pose.pose = msg.pose[23]
            self.wamv_pose.pose = msg.pose[22]
        else:
            self.current_pose.pose = msg.pose[22]
            self.wamv_pose.pose = msg.pose[23]

    def calculate_vel(self):
        dt = 1.0 / 30.0  # Assuming a constant loop rate of 30 Hz

        # Calculate position errors
        error_x = self.wamv_pose.pose.position.x - self.current_pose.pose.position.x
        error_y = self.wamv_pose.pose.position.y - self.current_pose.pose.position.y
        error_z = 4.0 - self.current_pose.pose.position.z  # Assuming a target altitude of 4.0 meters

        # Calculate yaw error
        (_, _, euler_uav_z) = euler_from_quaternion([self.current_pose.pose.orientation.x,
                                                      self.current_pose.pose.orientation.y,
                                                      self.current_pose.pose.orientation.z,
                                                      self.current_pose.pose.orientation.w])
        (_, _, euler_wamv_z) = euler_from_quaternion([self.wamv_pose.pose.orientation.x,
                                                       self.wamv_pose.pose.orientation.y,
                                                       self.wamv_pose.pose.orientation.z,
                                                       self.wamv_pose.pose.orientation.w])
        error_yaw = euler_wamv_z - euler_uav_z
        # rospy.loginfo("\n{},{},{},{}\n".format(error_x,error_y,error_z,error_yaw))

        # Compute PID control outputs
        self.current_vel.header.frame_id = "base_link"
        self.current_vel.twist.linear.x = self.pid_x.compute(error_x, dt)
        self.current_vel.twist.linear.y = self.pid_y.compute(error_y, dt)
        self.current_vel.twist.linear.z = self.pid_z.compute(error_z, dt)
        self.current_vel.twist.angular.z = self.pid_yaw.compute(error_yaw, dt)

        # Apply PID control outputs to the final velocity
        # self.current_vel.twist.linear.x = np.clip(control_output_x, -1.0, 1.0)
        # self.current_vel.twist.linear.y = np.clip(control_output_y, -1.0, 1.0)
        # self.current_vel.twist.linear.z = np.clip(control_output_z, -1.0, 1.0)
        # self.current_vel.twist.angular.z = np.clip(control_output_yaw, -1.0, 1.0)

if __name__ == '__main__':
    try:
        take_off = TakeOff()
    except rospy.ROSInterruptException:
        pass
