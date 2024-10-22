#!/usr/bin/env python3

import rospy
import cv2 as cv
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
        self.image_sub = rospy.Subscriber("/kevin/rgb/image_raw", Image, self.image_callback)

        # Publisher for the processed image
        self.image_pub = rospy.Publisher("/kevin/camera/rgb/image_raw/square", Image, queue_size=10)

        rospy.loginfo("Square Contour Detector Node Initialized")

    def image_callback(self, msg):
        try:
            # Convert the ROS image message to a CV2 image
            # np_arr = np.frombuffer(msg.data, np.uint8)
            # cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")

            # Process the image to find square contours
            processed_image = self.find_square_contours(cv_image)

            # Convert the processed OpenCV image to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")

            # Publish the processed image
            self.image_pub.publish(ros_image)

        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def find_square_contours(self, image):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray,(81,81),0)
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        ret2, thresh = cv.threshold(blur, 150, 255, cv.THRESH_BINARY)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        cnt = contours[0]
        # Approximate the contour to check if it's a square
        epsilon = 0.05 * cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, epsilon, True)

        # A square has 4 sides and is convex
        if len(approx) == 4 and cv.isContourConvex(approx):
            # Compute the bounding box and check for square-like dimensions
            x, y, w, h = cv.boundingRect(approx)
            aspect_ratio = w / float(h)

            if 0.9 <= aspect_ratio <= 1.1:  # Aspect ratio close to 1
                # Create a mask from the contour
                mask = np.zeros_like(gray)
                cv.fillPoly(mask, [approx], 255)

                # Extract pixel values from the mask
                masked_pixels = cv.bitwise_and(gray, gray, mask=mask)
                mean_pixel_value = cv.mean(masked_pixels, mask=mask)[0]  # Get mean pixel intensity

                # Check if pixel intensity is near the threshold (e.g., 200)
                if 200 <= mean_pixel_value <= 255:  # Range near 200
                    # Draw the contour as the pixel intensity condition is satisfied
                    cv.drawContours(image, [approx], 0, (0, 0, 255), 3)

        # Return the processed image
        return image

if __name__ == '__main__':
    try:
        # Initialize and run the detector
        SquareContourDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
