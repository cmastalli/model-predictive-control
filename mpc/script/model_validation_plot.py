from matplotlib.pyplot import figure, show
from pylab import *
from numpy import *


path = '/home/rene/ros_workspace/model-predictive-control/mpc/data/validation_output_data.txt'
t, x, y, z, u, v, w, roll, pitch, yaw, p, q, r = loadtxt(path, skiprows=2, unpack=True)

path2 = '/home/rene/ros_workspace/model-predictive-control/mpc/data/validation_input_data.txt'
tu, w1, w2, w3, w4 = loadtxt(path2, skiprows=2, unpack=True)

path3 = '/home/rene/ros_workspace/model-predictive-control/mpc/data/validation_nonlinear_data.txt'
tn, xn, yn, zn, un, vn, wn, rolln, pitchn, yawn, pn, qn, rn = loadtxt(path3, skiprows=2, unpack=True)

path4 = '/home/rene/ros_workspace/model-predictive-control/mpc/data/validation_torque_data.txt'
tt, U1, U2, U3, U4 = loadtxt(path4, skiprows=2, unpack=True)

# Estados

#Posiciones
figure(num=None, figsize=(8, 6))
subplot(311)
plot(tn, xn, '--k', linewidth=4)
plot(t, x, '#66ff00', linewidth=2.5)
ylabel('$x(t)$ $[m]$', {'color':'k', 'fontsize':16})

subplot(312)
plot(tn, yn, '--k', linewidth=4)
plot(t, y, '#66ff00', linewidth=2.5)
ylabel('$y(t)$ $[m]$', {'color':'k', 'fontsize':16})

subplot(313)
plot(tn, zn, '--k', linewidth=4)
plot(t, z, '#66ff00', linewidth=2.5)
ylabel('$z(t)$ $[m]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Non-linear$', r'$Linearized$'), shadow = True, loc = 'upper right')

#Velocidades
figure(num=None, figsize=(8,6))
subplot(311)
plot(tn, un, '--k', linewidth=4)
plot(t, u, '#66ff00', linewidth=2.5)
ylabel('$u(t)$ $[m/s]$', {'color':'k', 'fontsize':16})

subplot(312)
plot(tn, vn, '--k', linewidth=4)
plot(t, v, '#66ff00', linewidth=2.5)
ylabel('$v(t)$ $[m/s]$', {'color':'k', 'fontsize':16})

subplot(313)
plot(tn, wn, '--k', linewidth=4)
plot(t, w, '#66ff00', linewidth=2.5)
ylabel('$w(t)$ $[m/s]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Non-linear$', r'$Linearized$'), shadow = True, loc = 'upper right')

#Angulos de Euler
figure(num=None, figsize=(8,6))
subplot(311)
plot(tn, rolln, '--k', linewidth=4)
plot(t, roll, '#66ff00', linewidth=2.5)
ylabel('$\phi (t)$ $[rad]$', {'color':'k', 'fontsize':16})

subplot(312)
plot(tn, pitchn, '--k', linewidth=4)
plot(t, pitch, '#66ff00', linewidth=2.5)
ylabel('$\Theta (t)$ $[rad]$', {'color':'k', 'fontsize':16})

subplot(313)
plot(tn, yawn, '--k', linewidth=4)
plot(t, yaw, '#66ff00', linewidth=2.5)
ylabel('$\psi (t)$ $[rad]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Non-linear$', r'$Linearized$'), shadow = True, loc = 'upper right')

#Velocidades angulares
figure(num=None, figsize=(8,6))
subplot(311)
plot(tn, pn, '--k', linewidth=4)
plot(t, p, '#66ff00', linewidth=2.5)
ylabel('$p(t)$ $[rad/s]$', {'color':'k', 'fontsize':16})

subplot(312)
plot(tn, qn, '--k', linewidth=4)
plot(t, q, '#66ff00', linewidth=2.5)
ylabel('$q(t)$ $[rad/s]$', {'color':'k', 'fontsize':16})


subplot(313)
plot(tn, rn, '--k', linewidth=4)
plot(t, r, '#66ff00', linewidth=2.5)
ylabel('$r(t)$ $[rad/s]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Non-linear$', r'$Linearized$'), shadow = True, loc = 'upper right')


# Senal de Control
figure(num=None, figsize=(8, 6))
subplot(411)
plot(tu, w1, 'k', linewidth=2.5)
ylabel('$\omega_1 (t)$ $[rad/s]$', {'color':'k', 'fontsize':16})

subplot(412)
plot(tu, w2, 'k', linewidth=2.5)
ylabel('$\omega_2 (t)$ $[rad/s]$', {'color':'k', 'fontsize':16})

subplot(413)
plot(tu, w3, 'k', linewidth=2.5)
ylabel('$\omega_3 (t)$ $[rad/s]$', {'color':'k', 'fontsize':16})

subplot(414)
plot(tu, w4, 'k', linewidth=2.5)
ylabel('$\omega_4 (t)$ $[rad/s]$', {'color':'k', 'fontsize':16})


# Torques de Control
figure(num=None, figsize=(8, 6))
subplot(411)
plot(tt, U1, 'k', linewidth=2.5)
ylabel('$U_1 (t)$ $[N]$', {'color':'k', 'fontsize':16})

subplot(412)
plot(tt, U2, 'k', linewidth=2.5)
ylabel('$U_2 (t)$ $[N-m]$', {'color':'k', 'fontsize':16})

subplot(413)
plot(tt, U3, 'k', linewidth=2.5)
ylabel('$U_3 (t)$ $[N-m]$', {'color':'k', 'fontsize':16})

subplot(414)
plot(tt, U4, 'k', linewidth=2.5)
ylabel('$U_4 (t)$ $[N-m]$', {'color':'k', 'fontsize':16})

show()
