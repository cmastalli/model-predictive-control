from matplotlib.pyplot import figure, show
from pylab import *
from numpy import *


path = '../data/validation/validation_output_data.txt'
t, x, y, z, u, v, w, roll, pitch, yaw, p, q, r = loadtxt(path, skiprows=2, unpack=True)

path2 = '../data/validation/validation_input_data.txt'
tu, w1, w2, w3, w4 = loadtxt(path2, skiprows=2, unpack=True)

# Estados

#Posiciones
figure(num=None, figsize=(8, 6))
subplot(311)
#plot(t, x_r, '--k', linewidth=4)
plot(t, x, '#66ff00', linewidth=2.5)
ylabel('$x(t)$ $[m]$', {'color':'k', 'fontsize':16})

subplot(312)
#plot(t, y_r, '--k', linewidth=4)
plot(t, y, '#66ff00', linewidth=2.5)
ylabel('$y(t)$ $[m]$', {'color':'k', 'fontsize':16})

subplot(313)
#plot(t, z_r, '--k', linewidth=4)
plot(t, z, '#66ff00', linewidth=2.5)
ylabel('$z(t)$ $[m]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#legend((r'$Referencia$', r'$Respuesta$'), shadow = True, loc = (0.8, 0))

#Velocidades
figure(num=None, figsize=(8,6))
subplot(311)
#plot(t, x_r, '--k', linewidth=4)
plot(t, u, '#66ff00', linewidth=2.5)
ylabel('$u(t)$ $[m/s]$', {'color':'k', 'fontsize':16})

subplot(312)
#plot(t, y_r, '--k', linewidth=4)
plot(t, v, '#66ff00', linewidth=2.5)
ylabel('$v(t)$ $[m/s]$', {'color':'k', 'fontsize':16})

subplot(313)
#plot(t, z_r, '--k', linewidth=4)
plot(t, w, '#66ff00', linewidth=2.5)
ylabel('$w(t)$ $[m/s]$', {'color':'k', 'fontsize':16})
xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#legend((r'$Referencia$', r'$Respuesta$'), shadow = True, loc = (0.8, 0))

#Angulos de Euler
#figure(num=None, figsize=(8,6))
##subplot(311)
#plot(t, x_r, '--k', linewidth=4)
#plot(t, roll, '#66ff00', linewidth=2.5)
#ylabel('$\phi (t)$ $[rad]$', {'color':'k', 'fontsize':16})


#subplot(312)
##plot(t, y_r, '--k', linewidth=4)
#plot(t, pitch, '#66ff00', linewidth=2.5)
#ylabel('$\Theta (t)$ $[rad]$', {'color':'k', 'fontsize':16})

#subplot(313)
##plot(t, z_r, '--k', linewidth=4)
#plot(t, yaw, '#66ff00', linewidth=2.5)
#ylabel('$\psi (t)$ $[rad]$', {'color':'k', 'fontsize':16})
#xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
##legend((r'$Referencia$', r'$Respuesta$'), shadow = True, loc = (0.8, 0))

#Velocidades angulares
#figure(num=None, figsize=(8,6))
#subplot(311)
##plot(t, x_r, '--k', linewidth=4)
#plot(t, p, '#66ff00', linewidth=2.5)
#ylabel('$p(t)$ $[rad/s]$', {'color':'k', 'fontsize':16})

#subplot(312)
##plot(t, y_r, '--k', linewidth=4)
#plot(t, q, '#66ff00', linewidth=2.5)
#ylabel('$q(t)$ $[rad/s]$', {'color':'k', 'fontsize':16})


#subplot(313)
##plot(t, z_r, '--k', linewidth=4)
#plot(t, r, '#66ff00', linewidth=2.5)
#ylabel('$r(t)$ $[rad/s]$', {'color':'k', 'fontsize':16})
#xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
##legend((r'$Referencia$', r'$Respuesta$'), shadow = True, loc = (0.8, 0))


# Senal de Control
figure(num=None, figsize=(8, 6))
subplot(411)
plot(tu, w1, 'k', linewidth=2.5)
ylabel('$U_1 (t)$ $[N]$', {'color':'k', 'fontsize':16})

subplot(412)
plot(tu, w2, 'k', linewidth=2.5)
ylabel('$U_2 (t)$ $[N-m]$', {'color':'k', 'fontsize':16})

subplot(413)
plot(tu, w3, 'k', linewidth=2.5)
ylabel('$U_3 (t)$ $[N-m]$', {'color':'k', 'fontsize':16})

subplot(414)
plot(tu, w4, 'k', linewidth=2.5)
ylabel('$U_4 (t)$ $[N-m]$', {'color':'k', 'fontsize':16})

show()
