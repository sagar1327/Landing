#!/usr/bin/env python3

import cv2 as cv
import numpy as np

class DetectSquare():
    """For developing purposes."""
    def __init__(self):
        cap = cv.VideoCapture("/home/sagar/personal_ws/src/Landing/offboard_py/src/UAV_Search_and_Report/waterTest1.mp4")
        if not cap.isOpened():
            print("Error opening the video file.")
            return

        while cap.isOpened():
            ret1, self.frame = cap.read()
            if not ret1:
                print("Error reading frame from the video.")
                break

            gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
            blur = cv.GaussianBlur(gray,(81,81),0)
            ret2, thresh = cv.threshold(blur, 150, 255, cv.THRESH_BINARY)

            # Find contours
            contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            cnt = contours[0]
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
                        cv.drawContours(self.frame, [approx], 0, (0, 0, 255), 3)
                        print(f"Detected square with mean pixel intensity: {mean_pixel_value}")
                        # cv.circle(self.frame, (approx[2][0]), 20, (0, 255, 0), 1)

            # Show the processed frame
            cv.imshow('Processed Image', self.frame)

            if cv.waitKey(1) == ord('q'):
                break

        cap.release()
        cv.destroyAllWindows()

if __name__ == "__main__":
    try:
        DetectSquare()
    except KeyboardInterrupt:
        pass
