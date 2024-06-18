#!/usr/bin/env python3

import rospy
import numpy as np
import json
# from matplotlib import pyplot as plt
# from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from pygeodesy.geoids import GeoidPGM

class Height:
    def __init__(self):
        self.height = [[], [], [], [], [], [], []]
        self.timestamps = [[], [], [], [], [], [], []]  # Store timestamps separately
        self.GPS_global_data = NavSatFix()
        self.GPS_local_data = Odometry()
        self.GPS_raw_fix = NavSatFix()
        self.imu_data = Imu()
        self.magnetometer_data = MagneticField()
        self.odometry = Odometry()
        self.pose_data = PoseStamped()
        self.rel_alt = Float64()
        self.initial_timestamp = np.ones(7) * np.nan
        self._egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

        rospy.init_node("height_estimate", anonymous=True)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, callback=self.GPS_global)
        rospy.Subscriber("/mavros/global_position/local", Odometry, callback=self.GPS_local)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, callback=self.GPS_fix)
        # rospy.Subscriber("/mavros/imu/data", Imu, callback=self.IMU)
        # rospy.Subscriber("/mavros/imu/mag", MagneticField, callback=self.Magnetometer)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, callback=self.Odometry)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.Pose)

        # Create a figure and axis for plotting
        # self.fig, self.ax = plt.subplots()
        # self.line1, = self.ax.plot([], [], label='GPS global')
        # self.line2, = self.ax.plot([], [], label='GPS local')
        # self.line4, = self.ax.plot([], [], label='Odometry')
        # self.line5, = self.ax.plot([], [], label='Pose')
        # self.ax.set_xlabel('Timestamp')
        # self.ax.set_ylabel('Height')
        # self.ax.set_title('Height vs Time')
        # self.ax.grid(True)

        # Create an animation to continuously update the plot
        # self.animation = FuncAnimation(self.fig, self.update_plot, interval=100, cache_frame_data=False)

        # Show the plot
        # self.ax.legend()
        # plt.show()

        rospy.on_shutdown(self.save_file)
            

    def GPS_global(self, msg):
        self.GPS_global_data = msg
        lat = self.GPS_global_data.latitude
        long = self.GPS_global_data.longitude
        if np.isnan(self.initial_timestamp[0]):
            self.initial_timestamp[0] = msg.header.stamp.to_sec()
        self.timestamps[0].append(msg.header.stamp.to_sec() - self.initial_timestamp[0])  # Store the timestamp
        self.height[0].append(self.GPS_global_data.altitude - self._egm96.height(lat, long))

    def GPS_local(self, msg):
        self.GPS_local_data = msg
        if np.isnan(self.initial_timestamp[1]):
            self.initial_timestamp[1] = msg.header.stamp.to_sec()
        self.timestamps[1].append(msg.header.stamp.to_sec() - self.initial_timestamp[1])  # Store the timestamp
        self.height[1].append(self.GPS_local_data.pose.pose.position.z)

    def GPS_fix(self, msg):
        self.GPS_raw_fix = msg
        lat = self.GPS_raw_fix.latitude
        long = self.GPS_raw_fix.longitude
        if np.isnan(self.initial_timestamp[2]):
            self.initial_timestamp[2] = msg.header.stamp.to_sec()
        self.timestamps[2].append(msg.header.stamp.to_sec() - self.initial_timestamp[2])  # Store the timestamp
        self.height[2].append(self.GPS_raw_fix.altitude - self._egm96.height(lat, long))

    def Odometry(self, msg):
        self.odometry = msg
        if np.isnan(self.initial_timestamp[5]):
            self.initial_timestamp[5] = msg.header.stamp.to_sec()
        self.timestamps[5].append(msg.header.stamp.to_sec() - self.initial_timestamp[5])  # Store the timestamp
        self.height[5].append(self.odometry.pose.pose.position.z)

    def Pose(self, msg):
        self.pose_data = msg
        if np.isnan(self.initial_timestamp[6]):
            self.initial_timestamp[6] = msg.header.stamp.to_sec()
        self.timestamps[6].append(msg.header.stamp.to_sec() - self.initial_timestamp[6])  # Store the timestamp
        self.height[6].append(self.pose_data.pose.position.z)

    # def update_plot(self, _):
        # Update the plot data using timestamps as x-axis
        # self.line1.set_data(self.timestamps[0], self.height[0])
        # self.line2.set_data(self.timestamps[1], self.height[1])
        # self.line4.set_data(self.timestamps[4], self.height[4])
        # self.line5.set_data(self.timestamps[5], self.height[5])
        # self.ax.relim()
        # self.ax.autoscale_view()

    def save_file(self):
        height_dic = {'GPS global':self.height[0],'GPS local':self.height[1],'GPS raw fix':self.height[2],'Odometry':self.height[5],'Pose':self.height[6]}
        time_dic = {'GPS global':self.timestamps[0],'GPS local':self.timestamps[1],'GPS raw fix':self.timestamps[2],'Odometry':self.timestamps[5],
                    'Pose':self.timestamps[6]}
        with open('/home/sagar/ROS/src/offboard_py/src/Height1.txt', 'w') as file:
            file.write(json.dumps(height_dic))
        with open('/home/sagar/ROS/src/offboard_py/src/Time1.txt', 'w') as file:
            file.write(json.dumps(time_dic))


if __name__ == "__main__":
    try:
        height_plotter = Height()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
