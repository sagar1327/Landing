import cv2 as cv


class Contours:
    """Findind contours in an image"""
    def __init__(self):
        img = cv.imread('OpenCV.png')
        img = cv.resize(img, None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)
        assert img is not None, "file could not be read, check with os.path.exists()"
        imgray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(imgray, 29, 255, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Image moment
        cnt = contours[0]
        m = cv.moments(cnt)
        print("Moment: {}".format(m))

        # Centroid
        cx = int(m['m10']/m['m00'])
        cy = int(m['m01']/m['m00'])
        print("Centroid: {}x{}".format(cx, cy))

        # Contour Area
        area = cv.contourArea(cnt)

        # Perimeter
        perimeter = cv.arcLength(cnt, True)
        print("Perimeter: {}".format(perimeter))

        cv.drawContours(img, contours, -1, (255, 255, 255), 3)
        cv.imshow("Display", img)
        cv.waitKey(0)


def main():
    Contours()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()

