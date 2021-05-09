# import matplotlib.pyplot as plt
# import numpy as np

# plt.ion()
# fig, ax = plt.subplots()
# x, y = [],[]
# sc = ax.scatter(x,y)
# plt.xlim(0,10)
# plt.ylim(0,10)

# plt.draw()
# for i in range(1000):
#     x.append(np.random.rand(1)*10)
#     y.append(np.random.rand(1)*10)
#     sc.set_offsets(np.c_[x,y])
#     fig.canvas.draw_idle()
#     plt.pause(0.1)

# # plt.waitforbuttonpress()





import matplotlib.pyplot as plt
import numpy as np



plt.ion()
fig, ax = plt.subplots(2)
sc = ax[0].scatter(x0s,y0s)
sc1 = ax[1].scatter(st_matrix[0][0], r1, c='red')
ax[0].set_xlim(0,10)
ax[0].set_ylim(0,10)
ax[0].set_title("Multilateration coordinates")
ax[1].set_xlim(0,200)
ax[1].set_ylim(0,200)
ax[1].set_title("Kalman output distance")
#    plt.xlim(0,200)
#    plt.ylim(0,200)
plt.draw()





ax[0].set_title("Multilateration coordinates: " + str(round(x0,2)) + " " + str(round(y0,2)) )
ax[1].set_title("Kalman output distance: " + str(round(st_matrix[0][0],2)))

sc.set_offsets(np.c_[x0s,y0s])
sc1.set_offsets(np.c_[st_matrix[0][0],r1])
fig.canvas.draw_idle()
plt.pause(0.01)
