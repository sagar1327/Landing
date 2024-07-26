#!/usr/bin/env python3

import rospy
import numpy as np
import json
import argparse
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseStamped
from offboard_py.msg import SetPoint
from mavros_msgs.msg import State
from std_msgs.msg import String

class Height:
    NUM_ELEMENTS = 2  # Number of elements in x, y, and z arrays

    def __init__(self):
        self.x_position = [[] for _ in range(self.NUM_ELEMENTS)]
        self.y_position = [[] for _ in range(self.NUM_ELEMENTS)]
        self.timestamps = [[] for _ in range(self.NUM_ELEMENTS)]
        self.initial_timestamp = np.full(self.NUM_ELEMENTS, np.nan)
        self.pose_data = PoseStamped()
        self.initial_position_x = None
        self.initial_position_y = None
        self.setpoint_data = SetPoint()
        self.state_msg = State()
        self.state_updated = False
        self.landing_seq_msg = String()

        rospy.init_node("position_estimate", anonymous=True)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.Pose)
        rospy.Subscriber("/kevin/pid/setpoint", SetPoint, callback=self.Setpoint)
        rospy.Subscriber("/mavros/state", State, callback=self.State)
        rospy.Subscriber("/kevin/landing/sequence", String, callback=self.landing_sequence)

        self.setup_plot()

        # rospy.on_shutdown(self.save_file)

    def setup_plot(self):
        self.fig1 = plt.figure()
        self.ax1 = self.fig1.add_subplot(1, 1, 1)
        self.lines1 = [self.ax1.plot([], [], label=topic)[0] for topic in ['Set Point', 'Current']]
        self.ax1.set_xlabel('Time')
        self.ax1.set_ylabel('X')
        self.ax1.set_title('Setpoint - X')
        self.ax1.grid(True)

        self.fig2 = plt.figure()
        self.ax2 = self.fig2.add_subplot(1, 1, 1)
        self.lines2 = [self.ax2.plot([], [], label=topic)[0] for topic in ['Set Point', 'Current']]
        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Y')
        self.ax2.set_title('Setpoint - Y')
        self.ax2.grid(True)

        self.animation1 = FuncAnimation(self.fig1, self.update_plot1, interval=100, cache_frame_data=False)
        self.ax1.legend()
        self.animation2 = FuncAnimation(self.fig2, self.update_plot2, interval=100, cache_frame_data=False)
        self.ax2.legend()
        plt.show()

    def Pose(self, msg):
        if self.state_msg.mode == "OFFBOARD":
            self.pose_data = msg
            if (self.initial_position_x is None and self.initial_position_y is None):
                self.initial_position_x = self.pose_data.pose.position.x
                self.initial_position_y = self.pose_data.pose.position.y
                print(f"Starting position: {self.initial_position_x}, {self.initial_position_y}")
                print(f"Final position: {self.initial_position_x + self.setpoint_data.setpoint.x}, {self.initial_position_y + self.setpoint_data.setpoint.y}")
                
            self.update_position_data(1, self.pose_data.pose.position)
            self.update_position_data(0, self.setpoint_data.setpoint)

    def Setpoint(self, msg):
        self.setpoint_data = msg

    def State(self, msg):
        self.state_msg = msg
        self.state_updated = True

    def landing_sequence(self, msg):
        self.landing_seq_msg = msg

    def update_position_data(self, index, position):
        if index == 1:
            self.x_position[index].append((position.x - self.initial_position_x))
            self.y_position[index].append((position.y - self.initial_position_y))
        else:
            self.x_position[index].append(position.x)
            self.y_position[index].append(position.y)

        if np.isnan(self.initial_timestamp[index]):
            self.initial_timestamp[index] = rospy.Time.now().to_sec()
        self.timestamps[index].append(rospy.Time.now().to_sec() - self.initial_timestamp[index])

    def update_plot1(self, _):
        for i, line in enumerate(self.lines1):
            line.set_data(self.timestamps[i], self.x_position[i])

        self.ax1.relim()
        self.ax1.autoscale_view()

    def update_plot2(self, _):
        for i, line in enumerate(self.lines2):
            line.set_data(self.timestamps[i], self.y_position[i])

        self.ax2.relim()
        self.ax2.autoscale_view()

    def save_file(self):
        path = '/home/sagar/catkin_ws/src/Landing/offboard_py/src/logs/Apriltag'
        parser = argparse.ArgumentParser(description='2D plot.')
        parser.add_argument('Date', type=str, help='Date of the test. Ex - 030823, March 8th 2023.')
        parser.add_argument('Time', type=float, help='Time of the test. Ex - 18.15.')
        args = parser.parse_args()

        xposition_dic = {'Set Point': self.x_position[0], 'Current': self.x_position[1]}
        yposition_dic = {'Set Point': self.y_position[0], 'Current': self.y_position[1]}
        time_dic = {'Set Point': self.timestamps[0], 'Current': self.timestamps[1]}
        
        with open('{Path}/Date:{Date}/x_position_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time), 'w') as file:
            file.write(json.dumps(xposition_dic))
        with open('{Path}/Date:{Date}/y_position_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time), 'w') as file:
            file.write(json.dumps(yposition_dic))
        with open('{Path}/Date:{Date}/time_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time), 'w') as file:
            file.write(json.dumps(time_dic))


if __name__ == "__main__":
    try:
        height_plotter = Height()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
