import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

path = '../data/ardrone_mpc_data.txt'
ind, x, y, z, u, v, w, roll, pitch, yaw, p, q, r, x_r, y_r, z_r, u_r, v_r, w_r, roll_r, pitch_r, yaw_r, p_r, q_r, r_r, w_1, w_2, w_3, w_4 = np.loadtxt(path, skiprows=1, unpack=True)
sampling_time = 0.0083
t = sampling_time*ind

#mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')
#theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
#z = np.linspace(-2, 2, 100)
#r = z**2 + 1
#x = r * np.sin(theta)
#y = r * np.cos(theta)
ax.plot(x, y, z, label='Simulated quadrotor')
ax.plot(x_r, y_r, z_r, label='Reference trajectory')
ax.legend()



plt.show()
