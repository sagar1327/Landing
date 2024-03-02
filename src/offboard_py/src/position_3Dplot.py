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
    x_file_path = '/home/sagar/ROS/src/offboard_py/src/x_position.txt'
    y_file_path = '/home/sagar/ROS/src/offboard_py/src/y_position.txt'
    z_file_path = '/home/sagar/ROS/src/offboard_py/src/z_position.txt'
    time_file_path = '/home/sagar/ROS/src/offboard_py/src/time.txt'

    Position2DPlots(x_file_path, y_file_path, z_file_path, time_file_path)
