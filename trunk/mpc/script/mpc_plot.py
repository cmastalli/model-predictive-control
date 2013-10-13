from matplotlib.pyplot import figure, show
from pylab import *
from numpy import *


path = '/home/rene/ros_workspace/model-predictive-control/mpc/data/ardrone_mpc_data.txt'
x, y, z, u, v, w, roll, pitch, yaw, p, q, r, x_r, y_r, z_r, u_r, v_r, w_r, roll_r, pitch_r, yaw_r, p_r, q_r, r_r, w_1, w_2, w_3, w_4 = loadtxt(path, skiprows=1, unpack=True)
sampling_time = 0.0083
t = arange(0.0, x.size*sampling_time, sampling_time)


# Estados
figure(num=None, figsize=(8, 6))
subplot(311)
plot(t, x_r, '--k', linewidth=4)
plot(t, x, '#66ff00', linewidth=2.5)
ylabel('$H_1(t)$ $[cm]$', {'color':'k', 'fontsize':16})
xlim((0,5))

subplot(312)
plot(t, y_r, '--k', linewidth=4)
plot(t, y, '#66ff00', linewidth=2.5)
ylabel('$H_1(t)$ $[cm]$', {'color':'k', 'fontsize':16})
xlim((0,5))

subplot(313)
plot(t, z_r, '--k', linewidth=4)
plot(t, z, '#66ff00', linewidth=2.5)
ylabel('$H_2(t)$ $[cm]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Referencia$', r'$Respuesta$'), shadow = True, loc = (0.8, 0))
xlim((0,5))


# Senal de Control
figure(num=None, figsize=(8, 6))
plot(t, w_1, 'k', linewidth=2.5)
ylabel('$u(t)$ $[V]$', {'color':'k', 'fontsize':16})

plot(t, w_2, 'k', linewidth=2.5)
ylabel('$u(t)$ $[V]$', {'color':'k', 'fontsize':16})

plot(t, w_3, 'k', linewidth=2.5)
ylabel('$u(t)$ $[V]$', {'color':'k', 'fontsize':16})

plot(t, w_4, 'k', linewidth=2.5)
ylabel('$u(t)$ $[V]$', {'color':'k', 'fontsize':16})

# Error de Control
figure(num=None, figsize=(8, 6))
subplot(211)
plot(t, x_r-x, 'k', linewidth=2.5)
#axhspan(-0.05, 0.05, facecolor='0.75', alpha=0.5)
ylabel('$e_{H_1}(t)$ $[cm]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
xlim((0, 5))

subplot(212)
plot(t, z_r-z, 'k', linewidth=2.5)
#axhspan(-5, 5, facecolor='0.75', alpha=0.5)
ylabel('$e_{H_2}(t)$ $[cm]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
xlim((0, 5))

show()
