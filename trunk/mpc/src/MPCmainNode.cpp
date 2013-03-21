#include <iostream>
#include "model.h"
#include "optimizer.h"
#include "simulator.h"
#include "model_predictive_control.h"
 
using namespace std;

/*
Definition of the general steps of the MPC algorithm for further addition of the required lines of code. 

1) Initialization and instantiation of all the variables and classes required in the algorithm.
	TODO:
	-Definition of global variables to be used and initialization of these.
	-Instantiation of the classes. 
	-Definition of the required pointer to the classes and variables to be passed by reference to other functions in the algorithm.
	

2) Obtain current process state and formulate the optimization problem.
	TODO:
	-Write the required code to subscribe to the IMU topic and get the current process state.
	-Define cost function. (Implicitly done in the previous step when instantiating the right class)
	-Define constraints. (Implicitly done in the previous step when instantiating the right class)
	-Obtain the parameters for the cost function from the model.
	
3) Obtain the optimal solution for the specified time horizon.
	TODO:
	-Obtain the predicted outputs of the system for the defined time horizon.
	-Pass the parameters obtained for the cost function obtained in the previous step to the solver.
	-Initiate the solver.
	-Get the solver solution stored in a global variable.

4) Apply the first component of the optimal solution to the system.
	TODO:
	-Save the first component of the optimal solution, correspondent to the current time step, in an auxiliar variable.
	-Write a publisher to send the control signal for the current time step to the thrusters topic.

5) Shift horizon and update the current state of the system.
	TODO:
	-Set x(t+1), u(t+1) as the new x(t) and u(t)
	-
	
