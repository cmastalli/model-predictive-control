# -*- coding: utf-8 -*-
import yaml
import scipy
import numpy as np

#from matplotlib.pyplot import *
from control.matlab import *

#from numpy import array
from scipy import linalg
from cvxopt import solvers, matrix


# Nullspace basis computations
def null(A, eps=1e-15):
    u, s, vh = linalg.svd(A)
    n = A.shape[1]   # the number of columns of A
    if len(s)<n:
        expanded_s = np.zeros(n, dtype = s.dtype)
        expanded_s[:len(s)] = s
        s = expanded_s
    null_mask = (s <= eps)
    null_space = np.compress(null_mask, vh, axis=0)
    return np.transpose(null_space)


# Theory and computation of disturbance invariant sets for discrete-time linear systems
# Ilya Kolmanovsky and Elmer G. Gilbert
def pdiff(F_u, h_u, F_v, h_v):
	N = h_u.size[0]
	h = matrix(0.0,(N,1))
	solvers.options['show_progress'] = False
	for i in range(0,N):
		sol = solvers.lp(-F_u[i,:].trans(),F_v,h_v)
		h[i,0] = -sol['primal objective']

	F_d = F_u;
	h_d = h_u - h;
	return F_d, h_d




name_file = 'lbmpc_parameters.yaml'
stream = file(name_file, 'w')


# Tanks system model
Ad = 0.178; At = 15.52; H1b = 8.; H2b = 8.; g = 9810.; beta = 3.96
A = np.array([[-Ad*np.sqrt(g)/(2*At*np.sqrt(H1b)), 0.0], [Ad*np.sqrt(g)/(2*At*np.sqrt(H1b)), -Ad*np.sqrt(g)/(2*At*np.sqrt(H1b))]])
B = np.array([[beta/At], [0.0]])
d = np.array([[0.0],[0.0]])
C = np.array([[0.0, 1.0]])


# Number of states, inputs and outputs
states = len(A)
inputs = len(B[0])
outputs = len(C)

# State error weight matrix
Q = np.array([[30.0, 0.0],[0.0, 30.0]])

# Input error weight matrix
R = np.array([[1.0]])

nominal_feedback_weight = 8.; max_admissible_weight = 100.

# Define a feedback policy K_nom used for terminal set computations:
K_nom, S_nom, E_nom = lqr(A, B, Q, np.dot(nominal_feedback_weight, R))

# Define a feedback policy K and corresponding terminal cost
K, S, E = lqr(A, B, Q, np.dot(max_admissible_weight, R))

P, L, G = dare(A + B * K, B, Q, np.dot(nominal_feedback_weight, R));


# Define polytopic constraints on input F_u*x <= h_u and state F_x*x <= h_x.  Also define model uncertainty as a F_g*x <= h_g
max_cmd = 10.; max_h1 = 25.; max_h2 = 27.
state_uncertainty = np.array([[0.1],[0.1]])
ub_u = np.array([[max_cmd]])#[[max_cmd],[max_cmd]]
lb_u = np.array([[0.]])#[[0],[0]]
F_u = np.concatenate((np.identity(inputs), -np.identity(inputs)), axis = 0)
h_u = np.concatenate((ub_u, lb_u), axis = 0)

ub_x = np.array([[max_h1],[max_h2]])
lb_x = np.array([[0.],[0.]])
F_x = np.concatenate((np.identity(states), -np.identity(states)), axis = 0)
h_x = np.concatenate((ub_x, lb_x), axis = 0)

ub_g = state_uncertainty
lb_g = state_uncertainty
F_g = np.concatenate((np.identity(states), -np.identity(states)), axis = 0)
h_g = np.concatenate((ub_g, lb_g), axis = 0)


#==========================================================================
# Generate steady-state parameterization and their projection matrices
#==========================================================================
M = np.concatenate((np.concatenate((A - np.identity(states), B, np.zeros(shape=(states, outputs))), axis = 1), np.concatenate((C, np.zeros(shape=(outputs, inputs)), -np.identity(outputs)), axis = 1)), axis = 0)

X_null = null(np.matrix(M))
Lambda = X_null[0:states,:]
Psi = X_null[states:states+inputs,:]
Xi = X_null[states+inputs:states+inputs+outputs,:]

# Solutions of M*[x;u;y] = [-d;0] are of the form
X_0, res, rank, sin_val = linalg.lstsq(M, np.concatenate((d,np.zeros(shape=(outputs,1))), axis = 0))
Lambda_0 = X_0[0:states,:]
Psi_0 = X_0[states:states+inputs,:]
Xi_0 = X_0[states+inputs:states+inputs+outputs,:]


#==========================================================================
# Compute maximally invariant set
#==========================================================================
F_xg = F_x - F_g
h_xg = h_x - h_g

F_w = np.concatenate((np.concatenate((np.concatenate((np.concatenate((np.concatenate((F_x, np.zeros(shape=(2*states,outputs))), axis = 1),np.concatenate((np.zeros(shape=(2*states,states)), np.dot(F_x,Lambda)), axis = 1)), axis = 0), np.concatenate((np.dot(F_u,K_nom),np.dot(F_u,Psi - np.dot(K_nom,Lambda))), axis = 1)), axis = 0), np.concatenate((np.zeros(shape=(2*inputs,states)),np.dot(F_u,Psi)), axis = 1)), axis = 0), np.concatenate((np.dot(F_xg,A+np.dot(B,K_nom)), np.dot(np.dot(F_xg,B),Psi-np.dot(K_nom,Lambda))), axis = 1)), axis = 0)
# F_w = [                    F_x  zeros(length_Fx, inputs);
#        zeros(length_Fx,states)                F_x*Lambda;
#                        F_u*K_t    F_u*(Psi-K_nom*Lambda);
#        zeros(length_Fu,states)                   F_u*Psi;
#              F_xg*(A+B*K_nom)  F_xg*B*(PSI-K_nom*Lambda)];

h_w = np.concatenate((h_x,h_x-np.dot(F_x,Lambda_0), h_u-np.dot(F_u,Psi_0-np.dot(K_nom,Lambda_0)), h_u-np.dot(F_u,Psi_0), h_xg-np.dot(F_xg,np.dot(B,Psi-np.dot(K_nom,Lambda_0)))), axis = 0)
# h_w = [                               h_x;
#                          h_x-F_x*Lambda_0;
#            h_u-F_u*(Psi_0-K_nom*Lambda_0);
#                             h_u-F_u*Psi_0;
#       h _xg-F_xg*B*(Psi_0-K_nom*Lambda_0)];


# Subtract out points due to disturbance (F_g,h_g)
F_v = np.concatenate((np.concatenate((F_g,np.zeros(shape=(2*states, inputs))), axis = 1), np.concatenate((np.zeros(shape=(inputs,states)), np.identity(inputs)), axis = 1), np.concatenate((np.zeros(shape=(inputs,states)), -np.identity(inputs)), axis = 1)), axis = 0)
# F_v = [                 F_g zeros(2*states,inputs);
#        zeros(inputs,states)            eye(inputs);
#        zeros(inputs,states)           -eye(inputs)]
h_v = np.concatenate((h_g, np.zeros(shape=(2*inputs,1))), axis = 0)
# h_v =[               h_g;
#      zeros(2*inputs,1)]);


F_w_N, h_w_N = pdiff(matrix(F_w), matrix(h_w), matrix(F_v), matrix(h_v))



#==========================================================================
# Print the parameters of Learning-based MPC into a yaml file
#==========================================================================
q = []; p = []; r = []; k_nom = []; k = []; fu = []; hu = []; fx = []; hx = []; fw = []; hw = []
for i in range(Q.shape[0]):
	for j in range(Q.shape[1]):
		q.append(np.asscalar(Q[i,j]))
		p.append(np.asscalar(P[i][j]))

for i in range(R.shape[0]):
	for j in range(R.shape[1]):
		r.append(np.asscalar(R[i,j]))

for i in range(K_nom.shape[1]):
	k_nom.append(np.asscalar(K_nom[0,i]))
	k.append(np.asscalar(K[0,i]))

for i in range(F_u.shape[0]):
	hu.append(np.asscalar(h_u[i,0]))
	for j in range(F_u.shape[1]):
		fu.append(np.asscalar(F_u[i,j]))

for i in range(F_x.shape[0]):
	hx.append(np.asscalar(h_x[i,0]))
	for j in range(F_x.shape[1]):
		fx.append(np.asscalar(F_x[i,j]))

for i in range(F_w_N.size[0]):
	hw.append(h_w_N[i,0])
	for j in range(F_w_N.size[1]):
		fw.append(F_w_N[i,j])


yaml.dump({'mpc': {'optimizer': {'states_error_weight_matrix': {'data': q}, 'input_error_weight_matrix': {'data': r}, 'terminal_state_weight_matrix': {'data': p}, 'set_feedback_policy': {'data': k_nom}, 'cost_feedback_policy': {'data': k}, 'constraints': {'input_constraint': {'F_u': fu, 'h_u': hu}, 'state_constraint': {'F_x': fx, 'h_x': hx}, 'terminal_state_constraint': {'F_w': fw,'h_w': hw}}}}}, stream)



