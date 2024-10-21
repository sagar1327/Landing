#!/usr/bin/env python3

import rospy
import cv2 as cv
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

class Talker():
    """Publishes a normal and a compressed image."""
    def __init__(self):
        rospy.init_node('rpi_img_pub', anonymous=True)
        self.comp_img_msg = CompressedImage()
        self.normal_img_msg = Image()
        self.comp_pub = rospy.Publisher('/kevin/camera/rgb/image_raw/compressed', CompressedImage, queue_size=1)
        self.normal_pub = rospy.Publisher('/kevin/camera/rgb/image_raw/', Image, queue_size=1)
        self.rate = rospy.Rate(60)  # 60hz
        self.bridge = CvBridge()

        # Open the camera device
        self.cap = cv.VideoCapture("/dev/video0", cv.CAP_V4L)
        if not self.cap.isOpened():
            rospy.logerr("Error opening the camera.")
            return

        while not rospy.is_shutdown():
            self.ret, self.frame = self.cap.read()
            if not self.ret:
                rospy.logerr("Error reading frame from the camera.")
                break

            # Convert the frame to a compressed and normal image message
            self.encoded_img = cv.imencode('.jpg', self.frame, [int(cv.IMWRITE_JPEG_QUALITY), 5])[1]  # Adjust quality here
            self.comp_img_msg.data = self.encoded_img.tostring()
            self.normal_img_msg = self.bridge.cv2_to_imgmsg(self.frame,"bgr8")

            # Publish the compressed and normal image
            self.comp_pub.publish(self.comp_img_msg)
            self.normal_pub.publish(self.normal_img_msg)
            rospy.loginfo_once("Now publishing...")
            self.rate.sleep()

        # Release the camera when the node is interrupted
        self.cap.release()

if __name__ == '__main__':
    try:
        Talker()
    except rospy.ROSInterruptException:
        pass
