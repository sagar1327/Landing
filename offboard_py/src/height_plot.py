import json
from matplotlib import pyplot as plt

class HeightPlots():
    def __init__(self):
        with open('/home/sagar/ROS/src/offboard_py/src/z_position2.txt', 'r') as file:
            self.height = json.loads(file.read())
        with open('/home/sagar/ROS/src/offboard_py/src/Time1.txt', 'r') as file:
            self.timestamps = json.loads(file.read())

        # Create a figure and axis for plotting
        self.fig, self.ax = plt.subplots()
        # self.line1, = self.ax.plot([], [], label='GPS global')
        # self.line2, = self.ax.plot([], [], label='GPS local')
        # self.line3, = self.ax.plot([], [], label='GPS raw fix')
        self.line6, = self.ax.plot([], [], label='Odometry')
        self.line7, = self.ax.plot([], [], label='Pose')
        self.ax.set_xlabel('Timestamp')
        self.ax.set_ylabel('Height')
        self.ax.set_title('Height vs Time')
        self.ax.grid(True)

        # Update the plot data using timestamps as x-axis
        # self.line1.set_data(self.timestamps['GPS global'], self.height['GPS global'])
        # self.line2.set_data(self.timestamps['GPS local'], self.height['GPS local'])
        # self.line3.set_data(self.timestamps['GPS raw fix'], self.height['GPS raw fix'])
        self.line6.set_data(self.timestamps['Odometry'], self.height['Odometry'])
        self.line7.set_data(self.timestamps['Pose'], self.height['Pose'])
        
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.legend()
        plt.show()


if __name__ == '__main__':
    HeightPlots()