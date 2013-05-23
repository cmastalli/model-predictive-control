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
	int np = 5;		// prediction horizon size in samples
	int p = 1;		// number of inputs
	int q = 2;		// number of outputs
	double H[np*n*np*n];
	double F[np*n*np*n];	
	
	// Create the pointer to the Model class
	mpc::model::Model *model_ptr = new mpc::test_models::Tanksystem ();
	//mpc::optimizer::Optimizer *solver_ptr = new mpc::optimizer::QPOASES (model_ptr);

	solver_ptr->setOptimizationParams(n, np, p, H, F);
	for (int i=0; i<(np*p*np*p); i++) {
		std::cout<< "H["<< i <<"]:" << H[i] <<"and 		F["<< i <<"]:\n"<< F[i]<< std::endl;
	}


	return 0;
}

