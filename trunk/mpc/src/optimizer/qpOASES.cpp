#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <mpc/optimizer/QPOASES.h>
#include <mpc/model/model.h>
#include <mpc/optimizer/optimizer.h>

/********************************************************************

NOTE: in the mapping function specified below, the size of the correspondent matrix must be entered manually, since the Map function
cannot take a variable as an argument.

*********************************************************************/


bool mpc::optimizer::qpOASES::initSolver(int *nVar_,
										int *nConst_,
										double *H, 
										double* g, 
										double *G, 
										double *lb, 
										double *ub, 
										double *lbA, 
										double *ubA, 
										int &nWSR, 
										double *cputime)
{
	qpOASES::QProblem solver_(nVar_, nConst_);

	/* Solve first QP. */
	solver_.init( H,g,G,lb,ub,lbA,ubA, nWSR );

	return true;
}

void mpc::optimizer::qpOASES::hotstartSolver(double *g_new, 
											double *G_new, 
											double *lb_new, 
											double *ub_new, 
											double *lbA_new, 
											double *ubA_new, 
											int &nWSR, 
											double *cputime,
											double &optSol_)
{
	/* Solve second QP. */
	solver_.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

	/* Get and print solution of second QP. */
	double xOpt[mpc::STDMPC::nVar_];
	solver_.getPrimalSolution( xOpt );

	for (int i=0; i<nVar_; i++){
		std::cout <<"\nxOpt["<< i <<"] = "<< xOpt[i] << std::endl;
		optSol_[i] = xOpt[i];
	}

	
		
	std::cout <<"\nobjVal ="<< solver_.getObjVal() << std::endl;
	

}
