from matplotlib.pyplot import figure, show
from pylab import *
from numpy import *


path = '../data/ardrone_mpc_data.txt'
ind, x, y, z, u, v, w, roll, pitch, yaw, p, q, r, x_r, y_r, z_r, u_r, v_r, w_r, roll_r, pitch_r, yaw_r, p_r, q_r, r_r, w_1, w_2, w_3, w_4 = loadtxt(path, skiprows=1, unpack=True)
sampling_time = 0.0083
t = sampling_time*ind

# Estados
##Posiciones
figure(num=None, figsize=(8, 6))
subplot(311)
plot(t, x_r, '--k', linewidth=4)
plot(t, x, '#66ff00', linewidth=2.5)
ylabel('$x(t)$ $[m]$', {'color':'k', 'fontsize':16})
#xlim((0,5))

subplot(312)
plot(t, y_r, '--k', linewidth=4)
plot(t, y, '#66ff00', linewidth=2.5)
ylabel('$y(t)$ $[m]$', {'color':'k', 'fontsize':16})
#xlim((0,5))

subplot(313)
plot(t, z_r, '--k', linewidth=4)
plot(t, z, '#66ff00', linewidth=2.5)
ylabel('$z(t)$ $[m]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Reference$', r'$Response$'), shadow = True, loc = (0.8, 0))
#xlim((0,5))

##Velocidades
figure(num=None, figsize=(8, 6))
subplot(311)
plot(t, u_r, '--k', linewidth=4)
plot(t, u, '#66ff00', linewidth=2.5)
ylabel('$u(t)$ $[m/s]$', {'color':'k', 'fontsize':16})
#xlim((0,5))

subplot(312)
plot(t, v_r, '--k', linewidth=4)
plot(t, v, '#66ff00', linewidth=2.5)
ylabel('$v(t)$ $[m/s]$', {'color':'k', 'fontsize':16})
#xlim((0,5))

subplot(313)
plot(t, w_r, '--k', linewidth=4)
plot(t, w, '#66ff00', linewidth=2.5)
ylabel('$w(t)$ $[m/s]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Reference$', r'$Response$'), shadow = True, loc = (0.8, 0))
#xlim((0,5))

##Angulos de Euler
figure(num=None, figsize=(8, 6))
subplot(311)
plot(t, roll_r, '--k', linewidth=4)
plot(t, roll, '#66ff00', linewidth=2.5)
ylabel('$\phi (t)$ $[rad]$', {'color':'k', 'fontsize':16})
#xlim((0,5))

subplot(312)
plot(t, pitch_r, '--k', linewidth=4)
plot(t, pitch, '#66ff00', linewidth=2.5)
ylabel('$\omega (t)$ $[rad]$', {'color':'k', 'fontsize':16})
#xlim((0,5))

subplot(313)
plot(t, yaw_r, '--k', linewidth=4)
plot(t, yaw, '#66ff00', linewidth=2.5)
ylabel('$\psi (t)$ $[rad]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Reference$', r'$Response$'), shadow = True, loc = (0.8, 0))
#xlim((0,5))

##Velocidades angulares
figure(num=None, figsize=(8, 6))
subplot(311)
plot(t, p_r, '--k', linewidth=4)
plot(t, p, '#66ff00', linewidth=2.5)
ylabel('$p(t)$ $[rad/s]$', {'color':'k', 'fontsize':16})
#xlim((0,5))

subplot(312)
plot(t, q_r, '--k', linewidth=4)
plot(t, q, '#66ff00', linewidth=2.5)
ylabel('$q(t)$ $[rad/s]$', {'color':'k', 'fontsize':16})
#xlim((0,5))

subplot(313)
plot(t, r_r, '--k', linewidth=4)
plot(t, r, '#66ff00', linewidth=2.5)
ylabel('$r(t)$ $[rad/s]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Reference$', r'$Response$'), shadow = True, loc = (0.8, 0))
#xlim((0,5))

# Senal de Control
figure(num=None, figsize=(8, 6))
subplot(411)
plot(t, w_1, 'k', linewidth=2.5)
ylabel('$\omega_{1} (t)$ $[rad/s]$', {'color':'k', 'fontsize':16})
subplot(412)
plot(t, w_2, 'k', linewidth=2.5)
ylabel('$\omega_{2} (t)$ $[rad/s]$', {'color':'k', 'fontsize':16})
subplot(413)
plot(t, w_3, 'k', linewidth=2.5)
ylabel('$\omega_{3} (t)$ $[rad/s]$', {'color':'k', 'fontsize':16})
subplot(414)
plot(t, w_4, 'k', linewidth=2.5)
ylabel('$\omega_{4} (t)$ $[rad/s]$', {'color':'k', 'fontsize':16})


show()
