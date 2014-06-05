#from matplotlib.pyplot import figure, show
#from pylab import *
#from numpy import *


import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

path = '../data/ardrone_mpc_data.txt'
ind, x, y, z, u, v, w, roll, pitch, yaw, p, q, r, x_r, y_r, z_r, u_r, v_r, w_r, roll_r, pitch_r, yaw_r, p_r, q_r, r_r, w_1, w_2, w_3, w_4 = np.loadtxt(path, skiprows=1, unpack=True)
sampling_time = 0.0083
t = sampling_time*ind


## Trayectoria en 2D, plano XY
plt.figure(num=None, figsize=(8, 6))
plt.plot(x_r, y_r, color='b', linewidth=4)
plt.plot(x, y, color='r', linewidth=3)
plt.xlabel('$y$ $[m]$', {'color':'k', 'fontsize':16})
plt.ylabel('$x$ $[m]$', {'color':'k', 'fontsize':16})
plt.xlim((-0.5,2))
plt.ylim((-2,0.5))


plt.show()
