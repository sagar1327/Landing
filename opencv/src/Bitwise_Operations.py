#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv


class BitWis:
    """Masking and adding one image over another."""
    def __init__(self):
        self.img1 = cv.imread("OpenCV.png")
        self.re_img1 = cv.resize(self.img1, None, fx=0.1, fy=0.1, interpolation=cv.INTER_AREA)
        self.img2 = cv.imread("python.png")
        self.re_img2 = cv.resize(self.img2, (1000, 1000), interpolation=cv.INTER_AREA)
        self.masking()

    def masking(self):
        """Masking process."""
        rows, cols, channels = self.re_img1.shape
        roi = self.re_img2[0:rows, 0:cols]

        # masking
        gray = cv.cvtColor(self.re_img1, cv.COLOR_BGR2GRAY)
        ret, mask = cv.threshold(gray, 10, 255, cv.THRESH_BINARY)
        mask_inv = cv.bitwise_not(mask)

        img2_bg = cv.bitwise_and(roi, roi, mask=mask_inv)
        img1_fg = cv.bitwise_and(self.re_img1, self.re_img1, mask=mask)

        comb = cv.add(img2_bg, img1_fg)
        self.re_img2[0:rows, 0:cols] = comb

        cv.imshow("Display", self.re_img2)
        cv.waitKey(0)
        cv.destroyAllWindows()


def main():
    BitWis()


if __name__ == "__main__":
    main()


