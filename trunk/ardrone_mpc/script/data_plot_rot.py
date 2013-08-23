from matplotlib.pyplot import figure, show
from pylab import *
from numpy import *



path1 = '/home/rene/ros_workspace/model-predictive-control/ardrone_mpc/data/test_3/Identification_Test_Yaw/identification_input_data.txt'
#x, y, z, roll, pitch, yaw, dx, dy, dz, dyaw = loadtxt(path, skiprows=1, unpack=True)
t, vx, vy, vz, dyaw = loadtxt(path1, skiprows=1, unpack=True)
path2 = '/home/rene/ros_workspace/model-predictive-control/ardrone_mpc/data/test_3/Identification_Test_Yaw/identification_output_data.txt'
t_out, x, y, z, vxo, vyo, vzo  = loadtxt(path2, skiprows=1, unpack=True)

path3 = '/home/rene/ros_workspace/model-predictive-control/ardrone_mpc/data/test_3/Identification_Test_Yaw/identification_output_data_rotations.txt'
t_out_rot, roll, pitch, yaw  = loadtxt(path3, skiprows=1, unpack=True)


print "%d lines in your choosen file" % len(open(path3).readlines())
data_num_1 = len(open(path1).readlines()) - 1

t_abs = t - 1339.1700
t_out_abs = t_out - 1339.1700
t_rot_abs = t_out_rot - 1339.1700

roll_rad = roll*(3.141592/180)
pitch_rad = pitch*(3.141592/180)
yaw_rad = yaw*(3.141592/180)

#----------------------------------------------------------Derivation of the velocity-------------------------------------------------
Vyaw = [0 for row in yaw_rad]

index = 1
for index in range(len(yaw_rad)):
	#if index == 0:
		#print index
		#h = t_rot_abs[index]
		#Vyaw[index] = yaw[index]/h
	#else:
	h = t_rot_abs[index] - t_rot_abs[index-1]
	Vyaw[index] = (yaw_rad[index] - yaw_rad[index-1])/h
	print yaw_rad[index], yaw_rad[index - 1] 


print "%d elements in t_out_rot" % len(t_out_rot)
print "%d elements in Vyaw" % len(Vyaw)

print "%f is the first element in t_rot_abs" % t_rot_abs[0]

#-------------------------------------------------------Low Pass Filter implementation-------------------------------------------------
#sampling time
Ts = 0.001 
#cutout frequency
wc = 60

i=0

Vyawf = [0 for row in Vyaw]

global y_old
global yf_old

for i in range(len(Vyaw) - 1):
	y_old = Vyaw[i]
	yf_old = Vyawf[i]

	yf = wc*Ts*y_old - wc*Ts*yf_old + yf_old
	Vyawf[i+1] = yf

	yf_old = Vyawf[i+1]
	y_old = Vyaw[i+1]


#t_out_rot.pop()
#print "%d elements in Vyaw" % len(Vyaw)
#Ts = 0.01
#tu = arange(0.0, ux.size*Ts*size(dx)/size(ux), Ts*size(dx)/size(ux))
#ts = arange(0.0, dx.size*Ts, Ts)

# plot
figure(num=None, figsize=(10, 10))
#subplot(411)
plot(t_abs, dyaw, 'k', linewidth=2.5)
plot(t_rot_abs, Vyawf, '#66ff00', linewidth=2.5)
ylabel('$\dot{\psi}(t)$ $[rad/s]$', {'color':'k', 'fontsize':16})
#xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
legend((r'$Input$ $\dot{\psi_r}(t)$', r'$Output$ $\dot{\psi}(t)$'), shadow = True, loc = (0.8, 0.1))
xlim((0,4))
ylim((-2,2))

#subplot(412)
#plot(t_abs, vy, 'k', linewidth=2.5)
#plot(t_out_abs, vyo, '#66ff00', linewidth=2.5)
#ylabel('$\dot{y}(t)$ $[m/s],$ $y(t)$ $[m]$', {'color':'k', 'fontsize':16})
#xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#legend((r'$Input$ $\dot{y}(t)$', r'$Output$ $\dot{y}(t)$'), shadow = True, loc = (0.8, 0.1))
#xlim((0,5))
#ylim((-1.5,1.5))

#subplot(413)
#plot(t_abs, vz, 'k', linewidth=2.5)
#plot(t_out_abs, vzo, '#66ff00', linewidth=2.5)
#ylabel('$\dot{z}(t)$ $[m/s],$ $z(t)$ $[m]$', {'color':'k', 'fontsize':16})
#xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#legend((r'$Input$ $\dot{z}(t)$', r'$Output$ $\dot{z}(t)$'), shadow = True, loc = (0.8, 0.1))
#xlim((0,10))
#ylim((-1.5,1.5))
#subplot(414)
#plot(tu, uyaw, 'k', linewidth=2.5)
#plot(ts, dyaw, '#66ff00', linewidth=2.5)
#ylabel('$\dot{\phi}(t)$ $[deg/s]$', {'color':'k', 'fontsize':16})
#xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
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
