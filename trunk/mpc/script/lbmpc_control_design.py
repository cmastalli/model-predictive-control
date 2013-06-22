# -*- coding: utf-8 -*-
"""
Created on Mon Jun 17 13:45:52 2013

@author: cmastalli
"""
import yaml
import scipy
#from matplotlib.pyplot import *
from control.matlab import *
from numpy import *
from scipy import linalg


	
def null(A, eps=1e-15):
    u, s, vh = linalg.svd(A)
    n = A.shape[1]   # the number of columns of A
    if len(s)<n:
        expanded_s = zeros(n, dtype = s.dtype)
        expanded_s[:len(s)] = s
        s = expanded_s
    null_mask = (s <= eps)
    null_space = compress(null_mask, vh, axis=0)
    return transpose(null_space)



name_file = 'lbmpc_parameters.yaml'
stream = file(name_file, 'w')


# Tanks system model
A = [[0.9992, 0], [-0.000803, 1.001]]
B = [[0.002551], [0]]
d = [[0],[0]]
C = [[1, 0]]


# Number of states, inputs and outputs
states = len(A)
inputs = len(B[0])
outputs = len(C)

# State error weight matrix
Q = [[30, 0],[0, 30]]

# Input error weight matrix
R = [[1]]

q = []
for i in range(len(Q)):
	for j in range(len(Q[0])):
		q.append(Q[i][j])

r = []
for i in range(len(R)):
	for j in range(len(R[0])):
		r.append(R[i][j])

nominal_feedback_weight = 8
max_admissible_weight = 100


# Define a feedback policy K_nom used for terminal set computations:
K_nom, S_nom, E_nom = lqr(A, B, Q, dot(nominal_feedback_weight, R))

# Define a feedback policy K and corresponding terminal cost
K, S, E = lqr(A, B, Q, dot(max_admissible_weight, R))

P = dare(A + B * K, B, Q, dot(nominal_feedback_weight, R));



# Define polytopic constraints on input F_u*x <= h_u and state F_x*x <= h_x.  Also define model uncertainty as a F_g*x <= h_g
max_cmd = 10
max_h1 = 25
max_h2 = 27
state_uncertainty = [[0.1],[0.1]]
ub_u = [[max_cmd],[max_cmd]]
lb_u = [[0],[0]]
F_u = concatenate((identity(inputs), -identity(inputs)), axis = 0)
h_u = concatenate((ub_u, lb_u), axis = 0)

ub_x = [[max_h1],[max_h2]]
lb_x = [[0],[0]]
F_x = concatenate((identity(states), -identity(states)), axis = 0)
h_x = concatenate((ub_x, lb_x), axis = 0)

ub_g = state_uncertainty
lb_g = state_uncertainty
F_g = concatenate((identity(states), -identity(states)), axis = 0)
h_g = concatenate((ub_g, lb_g), axis = 0)


#==========================================================================
# Generate steady-state parameterization and their projection matrices
#==========================================================================
M = concatenate((concatenate((A - identity(states), B, zeros(shape=(states, outputs))), axis = 1), concatenate((C, zeros(shape=(outputs, inputs)), -identity(outputs)), axis = 1)), axis = 0)

X_null = null(matrix(M))
Lambda = X_null[0:states,:]
Psi = X_null[states:states+inputs,:]
Xi = X_null[states+inputs:states+inputs+outputs,:]

# Solutions of M*[x;u;y] = [-d;0] are of the form
X_0, res, rank, sin_val = linalg.lstsq(M, concatenate((d,zeros(shape=(outputs,1))), axis = 0))
Lambda_0 = X_0[0:states,:]
Psi_0 = X_0[states:states+inputs,:]
Xi_0 = X_0[states+inputs:states+inputs+outputs,:]


#==========================================================================
# Compute maximally invariant set
#==========================================================================


#R = matrix([[1,0,0,0,-1,-1,-1,0,0,0],[0,1,0,0,1,0,0,-1,-1,0],[0,0,0,0,0,1,0,1,0,-1],[0,0,0,0,0,0,1,0,0,-1],[0,0,0,-1,0,0,0,0,0,1],[0,0,-1,0,0,0,0,0,1,1]])



yaml.dump({'mpc': {'optimizer': {'states_error_weight_matrix': {'data': q}, 'input_error_weight_matrix': {'data': r}}}}, stream)
#'terminal_state_weight_matrix': {'data': p},


