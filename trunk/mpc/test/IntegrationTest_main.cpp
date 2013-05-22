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
	int n = 2;
	int np = 1;
	int p = 5;
	double H_[np*n*np*n];
	double F_[np*n*np*n];	
	
	// Initialization of the corresponding classes
	//mpc::test_models::Tanksystem testTank();
	//mpc::optimizer::QPOASES testSolve();

	// Create the pointer to the Model class
	
	mpc::model::Model *model_ptr = new mpc::test_models::Tanksystem ();
	//mpc::optimizer::Optimizer *solver_ptr = new mpc::optimizer::QPOASES (model_ptr);

	//solver_ptr->setOptimizationParams(n, np, p, H_, F_);
	//for (int i=0; i<(np*p*np*p); i++){
	//std::cout<< "H_["<< i <<"]:" << H_[i] <<"and 		F_["<< i <<"]:\n"<< F_[i]<< std::endl;
	//}

	
	return 0;
}

