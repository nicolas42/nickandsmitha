# based on https://stackoverflow.com/questions/4098131/how-to-update-a-plot-in-matplotlib

import numpy as np
import matplotlib.pyplot as plt
import time
import random

# x = np.linspace(0, 6*np.pi, 100)
# y = np.sin(x)
plotcolor = ['blue','blue','blue','blue','blue','blue','blue','blue','red']
sizevalue = 200
x = [0,4,0,4,0,4,0,4, -1]
y = [0,0,3.5,3.5,4.5,4.5,8,8, -1]

# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()

fig = plt.figure(figsize=(20,20), dpi=80)
ax = fig.add_subplot(111)
# line1, = ax.plot(x, y, 'r-') # Returns a tuple of line objects, thus the comma
sc = ax.scatter(x, y, c=plotcolor, s=sizevalue) # Returns a tuple of line objects, thus the comma

plt.title("omg omg")
plt.xlabel("omg x")
plt.ylabel("omg y")

plt.grid()
plt.xlim(-2, 10)
plt.ylim(-2, 10)

while True:

    # line1.set_ydata(np.sin(x + phase))
    x[-1] = random.randint(1,4)
    y[-1] = random.randint(1,4)

    sc.set_offsets(np.c_[x, y])

    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(1)



# plotcolor = ['blue','blue','blue','blue','blue','blue','blue','blue']
# x = [0,4,0,4,0,4,0,4]
# y = [0,0,3.5,3.5,4.5,4.5,8,8]

# x.append(2)
# y.append(2)
# plotcolor.append('red')

# plt.title("omg omg")
# plt.xlabel("omg x")
# plt.ylabel("omg y")

# # plt.plot(x, y)
# # plt.scatter(val, val, s=sizevalues, c=plotcolor)
# plt.scatter(np.array(x), np.array(y), c=plotcolor)
# plt.grid()
# plt.xlim(-2, 10)
# plt.ylim(-2, 10)

# plt.show()

