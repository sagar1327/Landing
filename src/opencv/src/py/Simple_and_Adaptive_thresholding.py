#!/home/sagar/ROS/opencv/env/bin/python

import cv2 as cv
from matplotlib import pyplot as plt

class Thresholding:
    """Practicing image thresholding"""

    # '0' is to convert the image into gray scale, alternative, cv.IMREAD_GRAYSCALE
    img = cv.imread('Bottle.jpg', 0)
    ret, img_new1 = cv.threshold(img, 20, 255, cv.THRESH_BINARY)
    img_new2 = cv.adaptiveThreshold(img, 255, cv.THRESH_BINARY, cv.ADAPTIVE_THRESH_MEAN_C, 11, 2)

    images = [img_new1, img_new2]
    for i in range(2):
        plt.subplot(1,2,i+1)
        plt.imshow(images[i],'gray')
    plt.show()
    

if __name__ == 'main':
    Thresholding()
