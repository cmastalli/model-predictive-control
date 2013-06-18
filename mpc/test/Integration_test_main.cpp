#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>

#include <qpOASES.hpp>
#include "mpc/optimizer/qpOASES.h"
#include "mpc/mpc/stdmpc.h"
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
	Eigen::MatrixXd x_k(states,1);
	x_k(0) = 8;
	x_k(1) = 12;

	Eigen::MatrixXd x_ref(states,1);
	x_ref(0) = 7;
	x_ref(1) = 7;
	 
	
	// Create the pointer to the Model class
	ros::NodeHandle node_handle("mpc");
	mpc::model::Model *model_ptr = new mpc::example_models::TanksSystem(node_handle);
	mpc::ModelPredictiveControl *mpc_ptr = new mpc::STDMPC(node_handle, model_ptr);

	mpc_ptr->initMPC();
	mpc_ptr->updateMPC(x_k, x_ref);

	return 0;
}

