#!/bin/env python3

import argparse
import json
from matplotlib import pyplot as plt

class Position2DPlots:
    def __init__(self, x_file_path, y_file_path, z_file_path, time_file_path):
        self.label = ['GPS local', 'Odometry', 'Pose']
        self.line = [[], [], []]

        with open(x_file_path, 'r') as file:
            self.x_position = json.loads(file.read())
        with open(y_file_path, 'r') as file:
            self.y_position = json.loads(file.read())
        with open(z_file_path, 'r') as file:
            self.z_position = json.loads(file.read())

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1, projection='3d')
        self.setup_3d_plot(self.ax, self.x_position, self.y_position, self.z_position, 'X', 'Y', 'Z', 'Position')

        plt.show()

    def setup_3d_plot(self, ax, xvalues, yvalues, zvalues, xlabel, ylabel, zlabel, title):
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_zlabel(zlabel)
        ax.set_title(title)
        ax.grid(True)

        for i in range(len(self.label)):
            ax.plot(xvalues[self.label[i]], yvalues[self.label[i]], zvalues[self.label[i]], label=self.label[i])

        ax.legend()
        ax.axis('equal')

if __name__ == '__main__':
    path = '/home/sagar/ROS/src/offboard_py/src/logs'
    parser = argparse.ArgumentParser(description='2D plot.')
    parser.add_argument('Date', type=str, help='Date of the test. Ex - 030823, March 8th 2023.')
    parser.add_argument('Time', type=float, help='Time of the test. Ex - 18.15.')
    args = parser.parse_args()

    x_file_path = '{Path}/Date:{Date}/x_position_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time)
    y_file_path = '{Path}/Date:{Date}/y_position_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time)
    z_file_path = '{Path}/Date:{Date}/z_position_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time)
    time_file_path = '{Path}/Date:{Date}/time_{Date}_{Time}.txt'.format(Path=path,Date=args.Date,Time=args.Time)

    Position2DPlots(x_file_path, y_file_path, z_file_path, time_file_path)
