#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>

#include <qpOASES.hpp>
#include "mpc/optimizer/qpOASES.h"

#include "mpc/example_models/tanks_system.h"


/** Example for qpOASES main function using the QProblem class. */
int main(int argc, char **argv)
{
	// Creation of the global variables to be used
	int states = 2;		// number of states
	//int horizon = 5;		// prediction horizon size in samples
	//int inputs = 1;		// number of inputs
	//int q = 2;		// number of outputs

	
	ros::init(argc, argv, "mpc");
	Eigen::VectorXd x_k(states);
	x_k(0) = 8;
	x_k(1) = 12;

	Eigen::VectorXd x_ref(states);
	x_ref(0) = 7;
	x_ref(1) = 7;
	 
	
	// Create the pointer to the Model class
	ros::NodeHandle node_handle("mpc");
	mpc::model::Model *model_ptr = new mpc::example_models::TanksSystem(node_handle);
	mpc::optimizer::Optimizer *solver_ptr = new mpc::optimizer::qpOASES(node_handle, model_ptr);

	solver_ptr->computeMPC(x_k, x_ref);
	

	return 0;
}

