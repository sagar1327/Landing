#!/bin/env python3

import os
import json
from matplotlib import pyplot as plt

class Position2DPlots:
    def __init__(self, x_file_path, y_file_path, z_file_path, time_file_path):
        self.label = ['GPS local','Odometry', 'Pose', 'Pressure']
        self.line = [[], []]

        with open(x_file_path, 'r') as file:
            self.x_position = json.loads(file.read())
        with open(y_file_path, 'r') as file:
            self.y_position = json.loads(file.read())
        with open(z_file_path, 'r') as file:
            self.z_position = json.loads(file.read())
        with open(time_file_path, 'r') as file:
            self.time = json.loads(file.read())

        self.fig1 = plt.figure()
        self.fig2 = plt.figure()
        self.ax1 = self.fig1.add_subplot(1, 1, 1)
        self.setup_2d_plot(self.ax1, self.x_position, self.y_position, 'X', 'Y', 'Position')

        self.ax2 = self.fig2.add_subplot(1, 1, 1)
        self.setup_2d_plot(self.ax2, self.time, self.z_position, 'Time', 'Height', 'Height vs Time')

        plt.show()

    def setup_2d_plot(self, ax, xvalues, yvalues, xlabel, ylabel, title):
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True)

        for i in range(len(self.label)):
            ax.plot(xvalues[self.label[i]], yvalues[self.label[i]], label=self.label[i])

        ax.legend()
        # ax.axis('equal')

if __name__ == '__main__':
    cwd = os.getcwd()
    x_file_path = '{}/x_position.txt'.format(cwd)
    y_file_path = '{}/y_position.txt'.format(cwd)
    z_file_path = '{}/z_position.txt'.format(cwd)
    time_file_path = '{}/time.txt'.format(cwd)

    Position2DPlots(x_file_path, y_file_path, z_file_path, time_file_path)
