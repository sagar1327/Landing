#!/home/sagar/ROS/opencv/env/bin/python

import numpy as np
import math as m

class OTSU:
    """Basics of OTSU Thresholding"""

    # Thresholding value
    T = 100
    # np.ravel contsructs a flat array
    BG = np.ravel([20, 50, 40, 10, 28, 28, 30])
    FG = np.ravel([130, 100, 200, 100, 120, 150, 120, 200, 100])
    N = BG.size + FG.size

    wbg = BG.size/N
    wfg = FG.size/N
    x_mean_bg = BG.sum()/BG.size
    x_mean_fg = FG.sum()/FG.size

    var_bg = 0
    var_fg = 0

    for i in range(BG.size):
        var_bg += m.pow((BG[i] - x_mean_bg), 2)
    var_bg = var_bg/(N-1)

    for i in range(FG.size):
        var_fg += m.pow((FG[i] - x_mean_fg), 2)
    var_fg = var_fg/(N-1)
    var = m.sqrt(wbg*var_bg + wfg*var_fg)

    print(var)


if __name__ == 'main':
    OTSU()