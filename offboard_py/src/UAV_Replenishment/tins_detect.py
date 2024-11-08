
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
from time import time
from math import atan2, tan
from coordinate_transform import calculate_altitude

# ###Parameters
# cannyEdgeMaxThr = 40 #Max Thr for canny edge detection
# circleDetectThr = 35 #Threshold for circle detection
# size = 30           #Size of the circles (to be calculated)
# factor = 3.2          #Factor big circle diameter / small circle diameter
# rangePerc = 1.5     #This is the range the circles are expected to be in

#cap = cv.VideoCapture(0)
#plt.ion()

def calculate_error_image(circles, img_width, img_height, num_of_circles):
    """Calculate the error in the x and y direction from the center of the image. X ranges from -1 to 1 and y ranges from -0.75 to 0.75"""
    if num_of_circles == 2:
        cnt_x = (circles[0][0][0] + circles[0][1][0]) / 2
        cnt_y = (circles[0][0][1] + circles[0][1][1]) / 2
        center_xy = (cnt_x, cnt_y)
    else: #only one circle
        center_xy = (circles[0][0], circles[0][1])
    error_xy = ((center_xy[0] / img_width-0.5)*2, (center_xy[1] / img_height-0.5)*-1.5)  # calculate relative error in x and y direction
    return error_xy

def tins(frame, altitude, cam_hfov, circle_parameters_obj):
    """Detects tins in the image using altitude"""
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    saturation = frame_hsv[:,:,1]
    #blur = cv.medianBlur(saturation,3)
    blur = saturation

    cannyEdgeMaxThr = circle_parameters_obj.canny_max_threshold*4.4
    #Max Thr for canny edge detection (can be much higher due to using saturation for edge detection)
    circleDetectThr = circle_parameters_obj.hough_circle_detect_thr*0.25#Threshold for circle detection (Lower since less wrong edges)
    tolerance = 1.25     #This is the tolarance the circles are expected to be in

    ###Calculate the size of the tin relative to altitude and camera hfov
    dist_img_on_ground = tan(cam_hfov/2)*2*altitude
    actual_radius = circle_parameters_obj.tin_diameter/2
    rel_size = actual_radius/dist_img_on_ground #This is the size of the tin compared to the overall frame
    radius_pixel = rel_size*frame.shape[1] #This is the radius of the tins in pixels
    radii_tins = [int(radius_pixel/tolerance), int(radius_pixel*tolerance)] #Min and max radius for the tins

    edges = cv.Canny(blur,0.5*cannyEdgeMaxThr,cannyEdgeMaxThr) #Only for visual representation (hough already does this)
    tins = cv.HoughCircles(blur,cv.HOUGH_GRADIENT,1,50,
                                param1=cannyEdgeMaxThr,param2=circleDetectThr,minRadius=radii_tins[0],maxRadius=radii_tins[1])
    
    if tins is None:
        return None , edges, None
    
    tins = np.int16(np.around(tins))
    tins = tins[0] #remove redundant dimensions

    radii_tins = []
    errors = []
    for tin in tins:
        error_xy = calculate_error_image(circles=[tin], img_width=frame.shape[1], img_height=frame.shape[0],num_of_circles=1)
        errors.append(error_xy)
        radii_tins.append(tin[2])
        #This is drawn on orignal frame image passed to function and not a copy
        cv.circle(frame,(tin[0],tin[1]),tin[2],(0,0,0),2)
        # draw the center of the circle
        cv.circle(frame,(tin[0],tin[1]),2,(0,0,0),3)
    return errors, edges, radii_tins