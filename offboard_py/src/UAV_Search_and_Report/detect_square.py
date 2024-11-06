#!/usr/bin/env python3

########### Strategy ############
# Detect a square
# Find its center
# Convert UTM to LLA conversion
# Get that conversion and add lat/lon on the image


import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from offboard_py.msg import Center, SquareCenters
from sensor_msgs.msg import NavSatFix

class SquareContourDetector:
    def __init__(self):
        rospy.init_node('square_contour_detector', anonymous=True)

        self.sqs_center = SquareCenters()
        self.img_msg = CompressedImage()
        self.target_wp_msg = NavSatFix()
        self.sq_img_msg = Image()
        self.sq_comp_img_msg = CompressedImage()
        self.sq_center_msg = Center()

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/kevin/camera/rgb/image_raw/compressed", CompressedImage, self.image_callback)
        self.target_wp = rospy.Subscriber("/kevin/search_report/inidividual/target/wp", NavSatFix, self.targetWP)
        self.image_pub = rospy.Publisher("/kevin/camera/square/rgb/image_raw", Image, queue_size=1)
        self.com_img_pub = rospy.Publisher("/kevin/camera/square/rgb/image_raw/compressed", CompressedImage, queue_size=1)
        self.sq_center_pub = rospy.Publisher("/kevin/target/squares/center", Center, queue_size=1)

        self.img_msg_received = False
        self.target_msg_received = False
        self.rate = rospy.Rate(60)

        rospy.loginfo("Square Contour Detector Node Initialized")

    def image_callback(self, msg):
        self.img_msg = msg
        self.img_msg_received = True

    def targetWP(self, msg):
        self.target_wp_msg = msg
        self.target_msg_received = True

    def find_square_contours(image,sq_center_msg):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray,(81,81),0)
        _, thresh = cv.threshold(blur, 150, 255, cv.THRESH_BINARY)
        contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if len(contours)!=0:
            for cnt in contours:
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
                            M = cv.moments(cnt)
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])
                            sq_center_msg.x = cx
                            sq_center_msg.y = cy
                            cv.circle(image,(cx,cy),10,(0,255,0),2)

        # Return the processed image
        return image,sq_center_msg
    

def main():
    SCD = SquareContourDetector()
    
    while not rospy.is_shutdown():

        if SCD.img_msg_received and SCD.target_msg_received:
            cv_image = SCD.bridge.compressed_imgmsg_to_cv2(SCD.img_msg,"jpg")
            processed_image,SCD.sq_center_msg = SCD.find_square_contours(cv_image,SCD.sq_center_msg)
            SCD.ros_image = SCD.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
            encoded_img = cv.imencode('.jpg', processed_image, [int(cv.IMWRITE_JPEG_QUALITY), 1])[1]  # Adjust quality here  
            SCD.comp_img_msg.data = encoded_img.tostring()

        SCD.image_pub.publish(SCD.ros_image)
        SCD.com_img_pub.publish(SCD.comp_img_msg)
        SCD.sq_center_pub.publish(SCD.sq_center_msg)

        SCD.img_msg_received = False
        SCD.target_msg_received = False
        SCD.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
