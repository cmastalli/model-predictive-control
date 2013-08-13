from matplotlib.pyplot import figure, show
from pylab import *
from numpy import *


path = '/home/cmastalli/ros_workspace/model-predictive-control/ardrone_mpc/data/test_1/identification_output_data.txt'
#x, y, z, roll, pitch, yaw, dx, dy, dz, dyaw = loadtxt(path, skiprows=1, unpack=True)
dx, dy, dz, dyaw = loadtxt(path, skiprows=1, unpack=True)
path = '/home/cmastalli/ros_workspace/model-predictive-control/ardrone_mpc/data/test_1/identification_input_data.txt'
ux, uy, uz, uyaw = loadtxt(path, skiprows=1, unpack=True)


Ts = 0.01
tu = arange(0.0, ux.size*Ts*size(dx)/size(ux), Ts*size(dx)/size(ux))
ts = arange(0.0, dx.size*Ts, Ts)

# plot
figure(num=None, figsize=(10, 10))
subplot(411)
plot(tu, ux, 'k', linewidth=2.5)
plot(ts, dx, '#66ff00', linewidth=2.5)
ylabel('$\dot{x}(t)$ $[m/s]$', {'color':'k', 'fontsize':16})
#xlim((0,10))

subplot(412)
plot(tu, uy, 'k', linewidth=2.5)
plot(ts, dy, '#66ff00', linewidth=2.5)
ylabel('$\dot{y}(t)$ $[m/s]$', {'color':'k', 'fontsize':16})
#xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#legend((r'$Referencia$', r'$Respuesta$'), shadow = True, loc = (0.8, 0))
#xlim((0,10))

subplot(413)
plot(tu, uz, 'k', linewidth=2.5)
plot(ts, dz, '#66ff00', linewidth=2.5)
ylabel('$\dot{z}(t)$ $[m/s]$', {'color':'k', 'fontsize':16})
#xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#legend((r'$Referencia$', r'$Respuesta$'), shadow = True, loc = (0.8, 0))
#xlim((0,10))

subplot(414)
plot(tu, uyaw, 'k', linewidth=2.5)
plot(ts, dyaw, '#66ff00', linewidth=2.5)
ylabel('$\dot{\phi}(t)$ $[deg/s]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#legend((r'$Referencia$', r'$Respuesta$'), shadow = True, loc = (0.8, 0))
#xlim((0,5))


# Senal de Control
#figure()
#plot(t, u, 'k', linewidth=2.5)
#ylabel('$u(t)$ $[V]$', {'color':'k', 'fontsize':16})


# Error de Control
#figure(num=None, figsize=(8, 6))
#subplot(211)
#plot(t, hr_1-h_1, 'k', linewidth=2.5)
#axhspan(-0.05, 0.05, facecolor='0.75', alpha=0.5)
#ylabel('$e_{H_1}(t)$ $[cm]$', {'color':'k', 'fontsize':16})
#xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#xlim((0, 100))

#subplot(212)
#plot(t, hr_2-h_2, 'k', linewidth=2.5)
#axhspan(-5, 5, facecolor='0.75', alpha=0.5)
#ylabel('$e_{H_2}(t)$ $[cm]$', {'color':'k', 'fontsize':16})
#xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#xlim((0, 100))

show()
