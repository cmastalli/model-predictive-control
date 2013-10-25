from matplotlib.pyplot import figure, show
from pylab import *
from numpy import *


path2 = '/home/rene/ros_workspace/model-predictive-control/mpc/data/validation/validation_input_data.txt'
tu, w1, w2, w3, w4 = loadtxt(path2, skiprows=2, unpack=True)
#sampling_time = 0.0083
#t = arange(0.0, x.size*sampling_time, sampling_time)

#
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

show()
