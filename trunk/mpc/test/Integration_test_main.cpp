#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>

#include <qpOASES.hpp>
#include "mpc/optimizer/optimizer.h"
#include "mpc/optimizer/QPOASES.h"

#include "mpc/model/model.h"
#include "mpc/test_models/Tanksystem.h"


/** Example for qpOASES main function using the QProblem class. */
int main(int argc, char **argv)
{
	// Creation of the global variables to be used
	int n = 2;		// number of states
	int horizon = 5;		// prediction horizon size in samples
	int p = 1;		// number of inputs
	//int q = 2;		// number of outputs

	
	ros::init(argc, argv, "mpc");
	Eigen::VectorXd x_k(n);
	x_k(0) = 8;
	x_k(1) = 12;

	Eigen::VectorXd x_ref(n);
	x_ref(0) = 7;
	x_ref(1) = 7;
	 
	
	// Create the pointer to the Model class
	mpc::model::Model *model_ptr = new mpc::test_models::Tanksystem ();
	mpc::optimizer::Optimizer *solver_ptr = new mpc::optimizer::QPOASES (model_ptr, n, p, horizon);

	solver_ptr->computeMPC(x_k, x_ref);
	

	return 0;
}

