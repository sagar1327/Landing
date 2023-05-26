#!/home/sagar/openv/env/bin/pyhton

import numpy as np
import math as m
from matplotlib import pyplot as plt

sigma = np.array([0.3, 1, 2])
# print(sigma[2])
x = np.linspace(-10, 10, num=100)
Gf = np.zeros((100, 3))
# print(Gf)

for i in range(3):
    for j in range(100):
        Gf[j][i] = (1/(m.sqrt(2*m.pi)*sigma[i]))*m.exp(-(x[j]**2)/(2*(sigma[i]**2)))

for i in range(3):
    plt.subplot(1, 3, i+1)
    plt.plot(x, Gf[:, i])
plt.show()
