from ultralytics import YOLO
import cv2

# Load a pre-trained YOLOv8 model
model = YOLO('/home/sagar/personal_ws/src/Landing/offboard_py/src/UAV_Search_and_Report/train2/weights/best.pt')  # 'yolov8n.pt' is the smallest pre-trained model; you can replace it with other versions.

# Load an image
# images we confirmed: Sarthak - 1250, 1214, 1357 <-- **edge case**, 
image_path = '/home/sagar'
image = cv2.imread(image_path)

# Run inference
results = model.predict(image, show=True)
while True:
    key = cv2.waitKey(1) & 0xFF
    if key == ord('x'):  # Close the window when 'x' is pressed
        break

cv2.destroyAllWindows()


# # Print detected objects
# for result in results:
#     # boxes = result.boxes  # get bounding boxes
#     # get bounding boxes, class index, and class names
#     boxes = result.boxes.cpu().numpy()
    
#     classes = result.boxes.cls.tolist()
#     names = result.names
#     BOXES = result.boxes
#     # for box in boxes:
#     print(f"Class: {BOXES.cls}, Confidence: {BOXES.conf}, Box coordinates: {BOXES.xyxy}")

# # Display the image with detections
# print(results)
