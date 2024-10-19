#!/usr/bin/env python3

import rospy
import numpy as np
import json
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from pygeodesy.geoids import GeoidPGM

class Height:
    def __init__(self):
        self.x_position = [[],[],[]]
        self.y_position = [[],[],[]]
        self.z_position = [[],[],[]]
        self.timestamps = [[],[],[]]
        self.initial_timestamp = np.ones(7) * np.nan
        self.GPS_local_data = Odometry()
        self.odometry = Odometry()
        self.pose_data = PoseStamped()

        rospy.init_node("position_estimate", anonymous=True)
        rospy.Subscriber("/mavros/global_position/local", Odometry, callback=self.GPS_local)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, callback=self.Odometry)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.Pose)

        # Create a figure and axis for plotting
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1, projection='3d')
        self.line1, = self.ax.plot([],[],[], label='GPS local')
        self.line2, = self.ax.plot([],[],[], label='Odometry')
        self.line3, = self.ax.plot([],[],[], label='Pose')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Position')
        self.ax.grid(True)

        # Create an animation to continuously update the plot
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=100, cache_frame_data=False)

        # Show the plot
        self.ax.legend()
        plt.show()

        # rospy.on_shutdown(self.save_file)
            

    def GPS_local(self, msg):
        self.GPS_local_data = msg
        self.x_position[0].append(self.GPS_local_data.pose.pose.position.x)
        self.y_position[0].append(self.GPS_local_data.pose.pose.position.y)
        self.z_position[0].append(self.GPS_local_data.pose.pose.position.z)
        if np.isnan(self.initial_timestamp[0]):
            self.initial_timestamp[0] = msg.header.stamp.to_sec()
        self.timestamps[0].append(msg.header.stamp.to_sec() - self.initial_timestamp[0])

    def Odometry(self, msg):
        self.odometry = msg
        self.x_position[1].append(self.odometry.pose.pose.position.x)
        self.y_position[1].append(self.odometry.pose.pose.position.y)
        self.z_position[1].append(self.odometry.pose.pose.position.z)
        if np.isnan(self.initial_timestamp[1]):
            self.initial_timestamp[1] = msg.header.stamp.to_sec()
        self.timestamps[1].append(msg.header.stamp.to_sec() - self.initial_timestamp[1])

    def Pose(self, msg):
        self.pose_data = msg
        self.x_position[2].append(self.pose_data.pose.position.x)
        self.y_position[2].append(self.pose_data.pose.position.y)
        self.z_position[2].append(self.pose_data.pose.position.z)
        if np.isnan(self.initial_timestamp[2]):
            self.initial_timestamp[2] = msg.header.stamp.to_sec()
        self.timestamps[2].append(msg.header.stamp.to_sec() - self.initial_timestamp[2])

    def update_plot(self, _):
        self.line1.remove()
        self.line1 = self.ax.plot(self.x_position[0], self.y_position[0], self.z_position[0], label='GPS local', color='blue')[0]

        self.line2.remove()
        self.line2 = self.ax.plot(self.x_position[1], self.y_position[1], self.z_position[1], label='Odometry', color='green')[0]

        self.line3.remove()
        self.line3 = self.ax.plot(self.x_position[2], self.y_position[2], self.z_position[2], label='Pose', color='red')[0]

        self.ax.relim()
        self.ax.autoscale_view()


    # def save_file(self):
    #     xposition_dic = {'GPS_local':self.x_position[0],'Odometry':self.x_position[1],'Pose':self.x_position[2]}
    #     yposition_dic = {'GPS_local':self.y_position[0],'Odometry':self.y_position[1],'Pose':self.y_position[2]}
    #     zposition_dic = {'GPS_local':self.z_position[0],'Odometry':self.z_position[1],'Pose':self.z_position[2]}
    #     time_dic = {'GPS_local':self.timestamps[0],'Odometry':self.timestamps[1],'Pose':self.timestamps[2]}
    #     with open('/home/sagar/ROS/src/offboard_py/src/x_position.txt', 'w') as file:
    #         file.write(json.dumps(xposition_dic))
    #     with open('/home/sagar/ROS/src/offboard_py/src/y_position.txt', 'w') as file:
    #         file.write(json.dumps(yposition_dic))
    #     with open('/home/sagar/ROS/src/offboard_py/src/z_position.txt', 'w') as file:
    #         file.write(json.dumps(zposition_dic))
    #     with open('/home/sagar/ROS/src/offboard_py/src/time.txt', 'w') as file:
    #         file.write(json.dumps(time_dic))


if __name__ == "__main__":
    try:
        height_plotter = Height()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
