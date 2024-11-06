#!/usr/bin/env python3

import rospy
import cv2 as cv
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


class DetectLogo():
    
    def __init__(self):
        rospy.init_node("detect_logo", anonymous=True)
        self.model = YOLO('/home/sagar/personal_ws/src/Landing/offboard_py/src/UAV_Search_and_Report/train2/weights/best.pt')
        self.frame = []
        self.labelClass = String()
        self.bridge = CvBridge()
        self.ros_img_msg = Image()
        self.ros_comp_img_msg = CompressedImage()

        rospy.Subscriber("/kevin/camera/rgb/image_raw", Image, callback=self.camera_img)
        self.img_pub = rospy.Publisher("/kevin/camera/logoN/rgb/image_raw", Image, queue_size=1)
        self.img_comp_pub = rospy.Publisher("/kevin/camera/logoN/rgb/image_raw/compressed", CompressedImage, queue_size=1)
        self.target_class_pub = rospy.Publisher("/kevin/target/class", String, queue_size=1)

        self.camera_img_received = False

        self.rate = rospy.Rate(60)

    def camera_img(self, msg):
        self.frame =  self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.camera_img_received = True

    def draw_detections(self, results):
        self.labelClass = String()
        # Loop through each detection in results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Extract the bounding box coordinates, confidence score, and class label
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                confidence = box.conf[0]                # Confidence score
                class_id = int(box.cls[0])              # Class ID

                # Get the class name from the YOLO model's class names list
                class_name = self.model.names[class_id] if self.model.names else f"Class {class_id}"
                self.labelClass.data = class_name

                # Draw the bounding box on the frame
                cv.rectangle(self.frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Display class name and confidence on the frame
                label = f"{class_name}: {confidence:.2f}"
                cv.putText(self.frame, label, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


def main():
    DL = DetectLogo()

    while not rospy.is_shutdown():
        if DL.camera_img_received:
            results = DL.model.predict(DL.frame, conf=0.45, verbose=False)

            # Draw bounding boxes and confidence scores on the frame
            DL.draw_detections(results)

            DL.ros_img_msg = DL.bridge.cv2_to_imgmsg(DL.frame,"bgr8")
            encoded_img = cv.imencode('.jpg', DL.frame, [int(cv.IMWRITE_JPEG_QUALITY), 1])[1]  # Adjust quality here
            DL.ros_comp_img_msg.data = encoded_img.tobytes()

            DL.img_pub.publish(DL.ros_img_msg)
            DL.img_comp_pub.publish(DL.ros_comp_img_msg)
            DL.target_class_pub.publish(DL.labelClass)

        DL.rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass