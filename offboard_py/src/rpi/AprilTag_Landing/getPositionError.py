#/usr/bin/env python3

import rospy
import numpy as np
from offboard_py.msg import ArTag, ArTagAltitude
from geometry_msgs.msg import  PoseStamped

class GetPositionError():
    
    def __init__(self):
        rospy.init_node("get_position_error", anonymous=True)

        self.artag_msg = ArTag()
        self.artag_alt_msg = ArTagAltitude()
        self.current_pose = PoseStamped()
        self.current_deltaS = np.Inf
        self.angle = (0, 0, 0)

        rospy.Subscriber("/kevin/artag/info", ArTag, callback=self.artag)
        rospy.Subscriber("/kevin/artag/altitude", ArTagAltitude, callback=self.artag_alt)
        rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback=self.uav_pose)

        self.rate = rospy.Rate(5)

    def artag(self, msg):
        self.artag_msg = msg

    def artag_alt(self, msg):
        self.artag_alt_msg = msg

    def uav_pose(self, msg):
        self.current_pose = msg
        self.angle = euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])

    def calculatePositionError(self):
        apx = 55/640
        apy = 55/480
        tag_centers = self.artag_msg.centers
        tag_alt = self.artag_alt_msg.altitude

        # Image frame.
        desired_center = tag_centers[0]
        delta_pixel_x = desired_center.x - 320
        delta_pixel_y = desired_center.y - 240
        alpha = np.abs(delta_pixel_x)*apx*np.pi/180
        beta = np.abs(delta_pixel_y)*apy*np.pi/180
        deltax_img = np.sign(delta_pixel_x)*np.tan(alpha)*tag_alt
        deltay_img = -np.sign(delta_pixel_y)*np.tan(beta)*tag_alt
        deltaS_img = np.sqrt(np.square(deltax_img) + np.square(deltay_img))
        theta_img = np.arctan2(deltay_img, deltax_img)

        # Global frame.
        self.current_deltaS = deltaS_img

        # Calculate the angle between the estimated position and the target position
        theta_horizontal = theta_img+self.angle[2]-np.pi/2 #Angle to the target in x-y plane
        theta_vertical = np.arctan2(self.current_deltaS, tag_alt) #Angle to the target in relative to straight down plane

        deltax = self.current_deltaS*np.cos(theta_horizontal)
        deltay = self.current_deltaS*np.sin(theta_horizontal)

        return deltax, deltay


def main():
    GPE = GetPositionError()

    while not rospy.is_shutdown():

        deltax, deltay = GPE.calculatePositionError()
        print(f"DeltaX: {deltax}, DeltaY: {deltay}")

        GPE.rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except ropsy.ROSInterruptException:
        pass