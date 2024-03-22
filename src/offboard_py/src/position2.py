#!/usr/bin/env python3

import rospy
import os
import numpy as np
import json
import argparse
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, FluidPressure
# Module to convert the GPS ellipsoidal height to amsl height
# from pygeodesy.geoids import GeoidPGM

class Height:
    NUM_ELEMENTS = 4  # Number of elements in x, y, and z arrays

    def __init__(self):
        self.x_position = [[] for _ in range(self.NUM_ELEMENTS)]
        self.y_position = [[] for _ in range(self.NUM_ELEMENTS)]
        self.z_position = [[] for _ in range(self.NUM_ELEMENTS)]
        self.timestamps = [[] for _ in range(self.NUM_ELEMENTS)]
        self.initial_timestamp = np.full(self.NUM_ELEMENTS, np.nan)
        self.GPS_local_data = Odometry()
        self.odometry = Odometry()
        self.pose_data = PoseStamped()
        # self.GPS_fix_data = NavSatFix()
        self.pressure_data = FluidPressure()
        # self._egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)
        self.initial_pressure_altitude = 0

        rospy.init_node("position_estimate", anonymous=True)
        rospy.Subscriber("/mavros/global_position/local", Odometry, callback=self.GPS_local)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, callback=self.Odometry)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.Pose)
        # rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, callback=self.GPS_fix)
        rospy.Subscriber("/mavros/imu/static_pressure", FluidPressure, callback=self.Pressure)

        self.setup_plot()

        # rospy.on_shutdown(self.save_file)

    def setup_plot(self):
        self.fig1 = plt.figure()
        self.fig2 = plt.figure()
        self.ax1 = self.fig1.add_subplot(1, 1, 1)
        self.ax2 = self.fig2.add_subplot(1, 1, 1)
        self.lines1 = [self.ax1.plot([], [], label=topic)[0] for topic in ['GPS local', 'Odometry', 'Pose']]
        self.lines2 = [self.ax2.plot([], [], label=topic)[0] for topic in ['GPS local', 'Odometry', 'Pose', 'Pressure']]
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Position')
        self.ax1.grid(True)
        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Height')
        self.ax2.set_title('Height vs Time')
        self.ax2.grid(True)

        self.animation1 = FuncAnimation(self.fig1, self.update_plot1, interval=100, cache_frame_data=False)
        self.animation2 = FuncAnimation(self.fig2, self.update_plot2, interval=100, cache_frame_data=False)
        self.ax1.legend()
        self.ax2.legend()
        plt.show()

    def GPS_local(self, msg):
        self.GPS_local_data = msg
        self.update_position_data(0, self.GPS_local_data.pose.pose.position)

    def Odometry(self, msg):
        self.odometry = msg
        self.update_position_data(1, self.odometry.pose.pose.position)

    def Pose(self, msg):
        self.pose_data = msg
        self.update_position_data(2, self.pose_data.pose.position)

    # def GPS_fix(self, msg):
    #     self.GPS_fix_data = msg
    #     lat, long = self.GPS_fix_data.latitude, self.GPS_fix_data.longitude
    #     self.x_position[3].append(lat)
    #     self.y_position[3].append(long)
    #     self.z_position[3].append(self.GPS_fix_data.altitude - self._egm96.height(lat, long))

    #     if np.isnan(self.initial_timestamp[3]):
    #         self.initial_timestamp[3] = rospy.Time.now().to_sec()
    #     self.timestamps[3].append(rospy.Time.now().to_sec() - self.initial_timestamp[3])

    def Pressure(self, msg):
        self.pressure_data = msg
        Psl = 101325;T0 = 288.16;L = 0.00976;R = 8.314462618;g = 9.80665;M = 0.02896968
        h = (T0/L)*(1 - (self.pressure_data.fluid_pressure/Psl)**((R*L)/(g*M)))
        self.x_position[3].append(0)
        self.y_position[3].append(0)
        if self.initial_pressure_altitude == 0:
            self.initial_pressure_altitude = h
        self.z_position[3].append(h - self.initial_pressure_altitude)

        if np.isnan(self.initial_timestamp[3]):
            self.initial_timestamp[3] = rospy.Time.now().to_sec()
        self.timestamps[3].append(rospy.Time.now().to_sec() - self.initial_timestamp[3])
        

    def update_position_data(self, index, position):
        self.x_position[index].append(position.x)
        self.y_position[index].append(position.y)
        self.z_position[index].append(position.z)

        if np.isnan(self.initial_timestamp[index]):
            self.initial_timestamp[index] = rospy.Time.now().to_sec()
        self.timestamps[index].append(rospy.Time.now().to_sec() - self.initial_timestamp[index])

    def update_plot1(self, _):
        for i, line in enumerate(self.lines1):
            line.set_data(self.x_position[i], self.y_position[i])

        self.ax1.relim()
        self.ax1.autoscale_view()

    def update_plot2(self, _):
        for i, line in enumerate(self.lines2):
            line.set_data(self.timestamps[i], self.z_position[i])
        
        self.ax2.relim()
        self.ax2.autoscale_view()

    def save_file(self):
        path = '/home/sagar/ROS/src/offboard_py/src/logs'
        parser = argparse.ArgumentParser(description='2D plot.')
        parser.add_argument('Date', type=str, help='Date of the test. Ex - 030823, March 8th 2023.')
        parser.add_argument('Time', type=float, help='Time of the test. Ex - 18.15.')
        args = parser.parse_args()

        xposition_dic = {'GPS local': self.x_position[0], 'Odometry': self.x_position[1], 'Pose': self.x_position[2],
                         'Pressure':self.x_position[3]}
        yposition_dic = {'GPS local': self.y_position[0], 'Odometry': self.y_position[1], 'Pose': self.y_position[2],
                         'Pressure':self.y_position[3]}
        zposition_dic = {'GPS local': self.z_position[0], 'Odometry': self.z_position[1], 'Pose': self.z_position[2],
                         'Pressure':self.z_position[3]}
        time_dic = {'GPS local': self.timestamps[0], 'Odometry': self.timestamps[1], 'Pose': self.timestamps[2],
                    'Pressure':self.timestamps[3]}
        
        with open('{Path}/Date:{Date}/x_position_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time), 'w') as file:
            file.write(json.dumps(xposition_dic))
        with open('{Path}/Date:{Date}/y_position_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time), 'w') as file:
            file.write(json.dumps(yposition_dic))
        with open('{Path}/Date:{Date}/z_position_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time), 'w') as file:
            file.write(json.dumps(zposition_dic))
        with open('{Path}/Date:{Date}/time_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time), 'w') as file:
            file.write(json.dumps(time_dic))


if __name__ == "__main__":
    try:
        height_plotter = Height()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
