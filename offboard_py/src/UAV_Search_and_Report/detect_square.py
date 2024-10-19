#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

class SquareContourDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('square_contour_detector', anonymous=True)

        # Set up a CvBridge to convert between ROS image messages and OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the image topic (CompressedImage)
        self.image_sub = rospy.Subscriber("/kevin/camera/rgb/image_raw/compressed", CompressedImage, self.image_callback)

        # Publisher for the processed image
        self.image_pub = rospy.Publisher("/kevin/camera/rgb/image_raw/square", Image, queue_size=10)

        rospy.loginfo("Square Contour Detector Node Initialized")

    def image_callback(self, msg):
        try:
            # Convert the ROS image message to a CV2 image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Process the image to find square contours
            processed_image = self.find_square_contours(cv_image)

            # Convert the processed OpenCV image to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")

            # Publish the processed image
            self.image_pub.publish(ros_image)

        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def find_square_contours(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply edge detection using Canny
        edged = cv2.Canny(blurred, 50, 150)

        # Find contours in the edged image
        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        squares = []
        for contour in contours:
            # Approximate the contour to check if it's a square
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # A square has 4 sides and is convex
            if len(approx) == 4 and cv2.isContourConvex(approx):
                # Compute the bounding box and check for square-like dimensions
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)
                if 0.9 <= aspect_ratio <= 1.1:  # Aspect ratio close to 1
                    squares.append(approx)

        # Draw the detected squares on the image
        for square in squares:
            cv2.drawContours(image, [square], -1, (0, 255, 0), 3)

        # Return the processed image
        return image

if __name__ == '__main__':
    try:
        # Initialize and run the detector
        SquareContourDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
