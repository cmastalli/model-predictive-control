from matplotlib.pyplot import figure, show
from pylab import *
from numpy import *


path = '/home/cmastalli/ros_workspace/model-predictive-control/mpc/data/tanks_stdmpc.txt'
h_1, h_2, hr_1, hr_2, u = loadtxt(path, skiprows=1, unpack=True)
sampling_time = 0.01
t = arange(0.0, h_1.size*sampling_time, sampling_time)


# Estados
figure(num=None, figsize=(8, 6))
subplot(211)
plot(t, hr_1, '--k', linewidth=4)
plot(t, h_1, '#66ff00', linewidth=2.5)
ylabel('$H_1(t)$ $[cm]$', {'color':'k', 'fontsize':16})
xlim((0,100))

subplot(212)
plot(t, hr_2, '--k', linewidth=4)
plot(t, h_2, '#66ff00', linewidth=2.5)
ylabel('$H_2(t)$ $[cm]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Referencia$', r'$Respuesta$'), shadow = True, loc = (0.8, 0))
xlim((0,100))


# Senal de Control
figure()
plot(t, u, 'k', linewidth=2.5)
ylabel('$u(t)$ $[V]$', {'color':'k', 'fontsize':16})


# Error de Control
figure(num=None, figsize=(8, 6))
subplot(211)
plot(t, hr_1-h_1, 'k', linewidth=2.5)
#axhspan(-0.05, 0.05, facecolor='0.75', alpha=0.5)
ylabel('$e_{H_1}(t)$ $[cm]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
xlim((0, 100))

subplot(212)
plot(t, hr_2-h_2, 'k', linewidth=2.5)
#axhspan(-5, 5, facecolor='0.75', alpha=0.5)
ylabel('$e_{H_2}(t)$ $[cm]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
xlim((0, 100))

show()
