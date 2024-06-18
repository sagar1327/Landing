#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import apriltag as ar
import cv2 as cv

class ApriltagDetector():
    
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = []
        options = ar.DetectorOptions(families=['tag36h11','tag25h9'],
                                    border=1,
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_blur=0.0,
                                    refine_edges=True,
                                    refine_decode=False,
                                    refine_pose=False,
                                    debug=False,
                                    quad_contours=True)
        self.detector = ar.Detector(options=options)
        rospy.init_node("air_tag", anonymous=True)
        rospy.Subscriber("/rpi/rgb/image_raw/compressed", CompressedImage, self.callback)
        self.tag_pub = rospy.Publisher("/artag/rgb/image_raw", Image, queue_size=1)
        self.rate = rospy.Rate(60)
        self.target_lost = False
        self.lost_time = None
        
        rospy.spin()

    def callback(self, msg):
        # rospy.loginfo("{}x{}\n".format(msg.height, msg.width))
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        if len(results) != 0:
            rospy.loginfo("Detected.")
            if self.target_lost:
            	rospy.loginfo(f"Lost target for: {rospy.Time.now().to_sec() - self.lost_time}")
            	self.target_lost = False
            	
            for r in results:
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                cv.line(self.cv_image, ptA, ptB, (0, 255, 0), 2)
                cv.line(self.cv_image, ptB, ptC, (0, 255, 0), 2)
                cv.line(self.cv_image, ptC, ptD, (0, 255, 0), 2)
                cv.line(self.cv_image, ptD, ptA, (0, 255, 0), 2)
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)
                # rospy.loginfo(r.center)
        else:
            if not self.target_lost:
                self.target_lost = True
                self.lost_time = rospy.Time.now().to_sec()
            rospy.loginfo("Not Detected.")

        img_msg = self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8')
        self.tag_pub.publish(img_msg)


if __name__ == '__main__':
    try:
        dt = ApriltagDetector()
    except rospy.ROSInterruptException:
        pass
