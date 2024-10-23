#!/usr/bin/env python3

import cv2 as cv
import numpy as np

class DetectSquare():
    """For developing purposes."""
    def __init__(self):
        self.frame = []
        self.cropped_image = []
        self.approx_cnt = []
        # self.rotated_image = []
        # self.cnt = []
        # self.cnt_rotated = []

        cap = cv.VideoCapture("/home/sagar/personal_ws/src/Landing/offboard_py/src/UAV_Search_and_Report/logoN.mp4")
        if not cap.isOpened():
            print("Error opening the video file.")
            return

        while cap.isOpened():
            ret, self.frame = cap.read()
            if not ret:
                print("Error reading frame from the video.")
                break

            # Get the ROI
            self.crop_ROI()
            # Rotate ROI
            # [self.rotated_image,self.cnt_rotated] = self.rotate_contour()
            # Identify the alphabet
            psuedoV = self.identifyChar()

            # Show the processed frame
            cv.imshow('Processed Image', self.frame)

            if cv.waitKey(1) == ord('q'):
                break

        cap.release()
        cv.destroyAllWindows()

    def crop_ROI(self):
        """Extract the ROI."""
        gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray,(81,81),0)
        _, thresh = cv.threshold(blur, 150, 255, cv.THRESH_BINARY)

        # Find contours
        contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            cnt = contours[0]
            epsilon = 0.05 * cv.arcLength(cnt, True)
            self.approx_cnt = cv.approxPolyDP(cnt, epsilon, True)

            # A square has 4 sides and is convex
            if len(self.approx_cnt) == 4 and cv.isContourConvex(self.approx_cnt):
                # Compute the bounding box and check for square-like dimensions
                x, y, w, h = cv.boundingRect(self.approx_cnt)
                aspect_ratio = w / float(h)

                if 0.9 <= aspect_ratio <= 1.1:  # Aspect ratio close to 1
                    # Create a mask from the contour
                    mask = np.zeros_like(gray)
                    cv.fillPoly(mask, [self.approx_cnt], 255)

                    # Extract pixel values from the mask
                    masked_pixels = cv.bitwise_and(gray, gray, mask=mask)
                    mean_pixel_value = cv.mean(masked_pixels, mask=mask)[0]  # Get mean pixel intensity

                    # Check if pixel intensity is near the threshold (e.g., 200)    
                    if 200 <= mean_pixel_value <= 255:  # Range near 200
                        # Draw the contour as the pixel intensity condition is satisfied
                        # cv.drawContours(self.frame, [approx], 0, (0, 0, 255), 3)

                        # Crop the region of interest (ROI)
                        self.cropped_image = self.frame[y:y+h,x:x+w]
                        # self.cropped_image = self.frame[int(np.ceil(h/2)+y)-25:int(np.ceil(h/2)+y)+25, int(np.ceil(x/2)+w)-25:int(np.ceil(x/2)+w)+25]
    
    def identifyChar(self):
        """Identify character as R or N."""
        gray = cv.cvtColor(self.cropped_image, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray, 150, 255, cv.THRESH_BINARY)
        contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        x, y, w, h = cv.boundingRect(self.approx_cnt)
        if len(contours)!=0:
            cnt = contours[0]
            epsilon = 0.05 * cv.arcLength(cnt, True)
            cnt = cv.approxPolyDP(cnt, epsilon, True)
            if len(cnt)==4 and cv.isContourConvex(cnt): 
                cnt_shifted = cnt + np.array([[[x,y]],
                                              [[x,y]],
                                              [[x,y]],
                                              [[x,y]]])
                cv.drawContours(self.frame, [cnt_shifted], 0, (0, 0, 255), 3)

        return thresh
    
    ## Function to rotate the contour to align with the image frame
    # def rotate_contour(self):
    #     # Get the minimum area bounding rectangle (rotated)
    #     rect = cv.minAreaRect(self.cnt)

    #     # Extract the center, width, height, and rotation angle from the rect
    #     (center, (width, height), angle) = rect

    #     # Get the rotation matrix for rotating the contour to align with the frame
    #     rotation_matrix = cv.getRotationMatrix2D(center, angle, 1.0)

    #     # Rotate the entire image
    #     rotated_image = cv.warpAffine(self.cropped_image, rotation_matrix, (self.cropped_image.shape[1], self.cropped_image.shape[0]))

    #     # Now transform the contour using the same rotation matrix
    #     contour_points = np.array([self.cnt], dtype=np.int32)
    #     contour_points = contour_points.reshape((-1, 1, 2))  # Reshape to (n_points, 1, 2)
    #     cnt_rotated = cv.transform(contour_points, rotation_matrix)

    #     return rotated_image, cnt_rotated

    ############## Usefull syntaxs ###########
    # 1. Draw circle: cv.circle(img, (x,y), 20, (0, 255, 0), 1)


if __name__ == "__main__":
    try:
        DetectSquare()
    except KeyboardInterrupt:
        pass
