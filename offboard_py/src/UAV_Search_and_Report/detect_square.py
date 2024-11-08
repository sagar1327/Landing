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
from std_msgs.msg import String
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
        self.target_class_msg = String()

        self.bridge = CvBridge()
        self.cv_image = []

        self.image_sub = rospy.Subscriber("/kevin/camera/rgb/image_raw/compressed", CompressedImage, self.image_callback)
        self.target_class_sub = rospy.Subscriber("/kevin/target/class",String, callback=self.target_class)
        self.target_wp = rospy.Subscriber("/kevin/search_report/inidividual/target/wp", NavSatFix, self.targetWP)
        self.image_pub = rospy.Publisher("/kevin/camera/square/rgb/image_raw", Image, queue_size=1)
        self.com_img_pub = rospy.Publisher("/kevin/camera/square/rgb/image_raw/compressed", CompressedImage, queue_size=1)
        self.sq_center_pub = rospy.Publisher("/kevin/target/squares/center", Center, queue_size=1)

        self.img_msg_received = False
        self.target_msg_received = False
        self.target_class_received = False
        self.initial_time = None
        self.class_count = 0
        self.rate = rospy.Rate(5)

        rospy.loginfo("Square Contour Detector Node Initialized")

    def image_callback(self, msg):
        self.img_msg = msg
        self.img_msg_received = True

    def targetWP(self, msg):
        self.target_wp_msg = msg
        self.target_msg_received = True

    def target_class(self, msg):
        self.target_class_msg = msg
        self.target_class_received = True

    def find_square_contours(self):
        gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray,(81,81),0)
        _, thresh = cv.threshold(blur, 150, 255, cv.THRESH_BINARY)
        contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        self.sq_center_msg = Center()

        # print("checkpoint 1")
        if len(contours)!=0:
            # print("checkpoint 2")
            for cnt in contours:
                cnt = contours[0]
                # Approximate the contour to check if it's a square
                epsilon = 0.05 * cv.arcLength(cnt, True)
                approx = cv.approxPolyDP(cnt, epsilon, True)

                # A square has 4 sides and is convex
                if len(approx) == 4 and cv.isContourConvex(approx):
                    # print("checkpoint 3")
                    # Compute the bounding box and check for square-like dimensions
                    x, y, w, h = cv.boundingRect(approx)
                    aspect_ratio = w / float(h)

                    if 0.9 <= aspect_ratio <= 1.1:  # Aspect ratio close to 1
                        # print("checkpoint 4")
                        # Create a mask from the contour
                        mask = np.zeros_like(gray)
                        cv.fillPoly(mask, [approx], 255)

                        # Extract pixel values from the mask
                        masked_pixels = cv.bitwise_and(gray, gray, mask=mask)
                        mean_pixel_value = cv.mean(masked_pixels, mask=mask)[0]  # Get mean pixel intensity
                        # print(mean_pixel_value)

                        # Check if pixel intensity is near the threshold (e.g., 200)
                        if 150 <= mean_pixel_value <= 255:  # Range near 200
                            # print("checkpoint 5")
                            # Draw the contour as the pixel intensity condition is satisfied
                            cv.drawContours(self.cv_image, [approx], 0, (0, 0, 255), 3)
                            M = cv.moments(cnt)
                            self.sq_center_msg.x = int(M['m10']/M['m00'])
                            self.sq_center_msg.y = int(M['m01']/M['m00'])
                            cv.circle(self.cv_image,(self.sq_center_msg.x,self.sq_center_msg.y),10,(0,255,0),2)

                            return 1
    

def main():
    SCD = SquareContourDetector()

    font = cv.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    color = (255, 255, 255)  # White color in BGR
    thickness = 1
    
    while not rospy.is_shutdown():

        if SCD.img_msg_received:
            SCD.cv_image = SCD.bridge.compressed_imgmsg_to_cv2(SCD.img_msg,"bgr8")
            ret = False
            ret = SCD.find_square_contours()

            if SCD.target_msg_received:
                lat = f"Lat: {SCD.target_wp_msg.latitude}"
                long = f"Long: {SCD.target_wp_msg.longitude}"
                cv.putText(SCD.cv_image,lat,(10,400),font,font_scale,color,thickness)
                cv.putText(SCD.cv_image,long,(10,420),font,font_scale,color,thickness)
            # print(SCD.target_class_msg)
            if ret and SCD.target_class_received:
                # print("checkpoint 6")
                if SCD.initial_time is None:
                    # print("checkpoint 1")
                    SCD.initial_time = rospy.Time.now().to_sec()

                if SCD.target_class_msg.data != "":
                    SCD.class_count += 1
                # print(rospy.Time.now().to_sec() - SCD.initial_time)
                if (rospy.Time.now().to_sec() - SCD.initial_time) > 1:
                    if SCD.class_count > 5:
                        print("Class of target: N")
                        SCD.initial_time = None
                        cv.imwrite("/home/sagar/N_target.png", SCD.cv_image)
                        # print("checkpoint 2")
                    else:
                        print("Class of target: R")
                        SCD.initial_time = None
                        cv.imwrite("/home/sagar/R_target.png", SCD.cv_image)
                        # print("checkpoint 3")


            SCD.sq_img_msg = SCD.bridge.cv2_to_imgmsg(SCD.cv_image, encoding="bgr8")
            encoded_img = cv.imencode('.jpg', SCD.cv_image, [int(cv.IMWRITE_JPEG_QUALITY), 1])[1]  # Adjust quality here  
            SCD.sq_comp_img_msg.data = encoded_img.tobytes()

        SCD.image_pub.publish(SCD.sq_img_msg)
        SCD.com_img_pub.publish(SCD.sq_comp_img_msg)
        # print(f"{SCD.sq_center_msg.x}, {SCD.sq_center_msg.y}")
        SCD.sq_center_pub.publish(SCD.sq_center_msg)

        SCD.img_msg_received = False
        SCD.target_msg_received = False
        SCD.target_class_received = False
        SCD.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
