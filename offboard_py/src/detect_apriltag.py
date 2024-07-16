#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from offboard_py.msg import ArTag, Center, ArTagAltitude
from cv_bridge import CvBridge
import apriltag as ar
import cv2 as cv
import json

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
        rospy.init_node("kevin_air_tag", anonymous=True)
        rospy.Subscriber("/kevin/camera/rgb/image_raw/compressed", CompressedImage, self.callback)
        self.tag_img_pub = rospy.Publisher("/kevin/artag/rgb/image_raw", Image, queue_size=1)
        self.tag_pub = rospy.Publisher("/kevin/artag", ArTag, queue_size=1)
        self.tag_altitude_pub = rospy.Publisher("/kevin/artag/altitude", ArTagAltitude, queue_size=1)
        self.rate = rospy.Rate(60)

        # Camera and Apriltag
        self.fov = 48.70141
        self.image_size = [640, 480]
        self.apx = self.fov/self.image_size[0]
        self.apy = self.fov/self.image_size[1]
        self.target_detected_previously = False
        self.target_lost = False
        self.lost_time = None
        self.losttime_list = []
        self.tag_altitude = ArTagAltitude()
        self.tag_altitude.header.frame_id = 'map'
        self.tag_altitude.altitude = np.Inf

        rospy.spin()

    def callback(self, msg):
        # rospy.loginfo("{}x{}\n".format(msg.height, msg.width))
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        # self.cv_image = cv.rotate(self.cv_image, cv.ROTATE_180)
        gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        altitude = []
        self.ArTag = ArTag()
        self.ArTag.header.frame_id = 'map'
        if len(results) != 0:
            # print("\nDetected.")
            if self.target_lost:
                self.losttime_list.append(rospy.Time.now().to_sec() - self.lost_time)
                print(f"\nLost target for: {rospy.Time.now().to_sec() - self.lost_time}")
                self.target_lost = False
	
            for r in results:
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                edge1_px = np.sqrt(np.square(ptA[0]-ptB[0]) + np.square(ptA[1]-ptB[1]))
                edge2_px = np.sqrt(np.square(ptB[0]-ptC[0]) + np.square(ptB[1]-ptC[1]))
                edge3_px = np.sqrt(np.square(ptC[0]-ptD[0]) + np.square(ptC[1]-ptD[1]))
                edge4_px = np.sqrt(np.square(ptD[0]-ptA[0]) + np.square(ptD[1]-ptA[1]))
                mean_edge_length_px = np.mean([edge1_px,edge2_px,edge3_px,edge4_px])

                delta_pixel_y = r.center[1] - self.image_size[1]/2
                beta = np.abs(delta_pixel_y)*self.apy*np.pi/180
                theta = np.abs(mean_edge_length_px)*self.apx*np.pi/180
                # print(theta)

                if r.tag_family.decode("utf-8") == "tag36h11":
                    actual_target_size = 0.25364
                elif r.tag_family.decode("utf-8") == "tag25h9":
                    actual_target_size = 0.162

                # altitude.append(np.sqrt(np.square(actual_target_size/(2*np.tan(theta/2)))/(np.square(np.tan(beta)+1))))
                altitude.append(actual_target_size/(2*np.tan(theta/2)))

                # print("\nTag: {}, Altitude: {}".format(r.tag_family.decode("utf-8"), altitude))

                cv.line(self.cv_image, ptA, ptB, (0, 255, 0), 2)
                cv.line(self.cv_image, ptB, ptC, (0, 255, 0), 2)
                cv.line(self.cv_image, ptC, ptD, (0, 255, 0), 2)
                cv.line(self.cv_image, ptD, ptA, (0, 255, 0), 2)
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)
                # rospy.loginfo(r.center)

                self.ArTag.family_names.append(str(r.tag_family.decode("utf-8")))
                center = Center()
                center.x = int(r.center[0])
                center.y = int(r.center[1])
                self.ArTag.centers.append(center)

            self.tag_altitude.altitude = np.mean(altitude)
            self.target_detected_previously = True
            self.ArTag.detected = True
            self.ArTag.previously_detected = True
            self.ArTag.total_tags = len(results)
            self.lost_time is None

        else:
            if self.target_detected_previously:
                if self.lost_time is None:
                    self.lost_time = rospy.Time.now().to_sec()
                self.ArTag.previously_detected = True
                self.ArTag.duration = rospy.Time.now().to_sec() - self.lost_time
            # else:    
            #     print("\nNot Detected.")

        img_msg = self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8')
        self.tag_img_pub.publish(img_msg)

        self.ArTag.header.stamp = rospy.Time.now()
        self.tag_pub.publish(self.ArTag)

        self.tag_altitude.header.stamp = rospy.Time.now()
        self.tag_altitude_pub.publish(self.tag_altitude)

    def save_file(self):
        with open('/home/sagar/catkin_ws/src/Landing/offboard_py/src/logs/Apriltag/target_lost_times.txt', 'w') as file:
            file.write(json.dumps(self.losttime_list))


# def main():
    # DT = ApriltagDetector()

    # rospy.spin()

    # while not rospy.is_shutdown():
    #     DT.ArTag.header.stamp = rospy.Time.now()
    #     DT.ArTag.header.frame_id = 'map'
    #     DT.tag_pub.publish(DT.ArTag)

    #     DT.tag_altitude.header.stamp = rospy.Time.now()
    #     DT.tag_altitude.header.frame_id = 'map'
    #     DT.tag_altitude_pub.publish(DT.tag_altitude)

    #     DT.rate.sleep()

        # rospy.on_shutdown(self.save_file)

if __name__ == '__main__':
    try:
        # main()
        ApriltagDetector()
    except rospy.ROSInterruptException:
        pass
