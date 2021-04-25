import numpy as np
import matplotlib.pyplot as plt

plt.axis([0, 4, 0, 4])

for i in range(10):
    x = 4*np.random.random()
    y = 4*np.random.random()
    plt.scatter(x, y)
    plt.pause(0.05)

plt.show()
