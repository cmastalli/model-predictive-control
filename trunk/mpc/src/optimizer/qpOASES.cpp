#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <mpc/optimizer/qpOASES.h>
#include <mpc/model/model.h>
#include <mpc/optimizer/optimizer.h>

/********************************************************************

NOTE: in the mapping function specified below, the size of the correspondent matrix must be entered manually, since the Map function
cannot take a variable as an argument.

*********************************************************************/
USING_NAMESPACE_QPOASES

mpc::optimizer::qpOASES::qpOASES(nVar, nConst, horizon, inputs)
{
nVar_ = nVar;
nConst_ = nConst;
horizon_ = horizon;
inputs_ = inputs;

}


bool mpc::optimizer::qpOASES::initSolver(double *H, 
										 double* g, 
										 double *G, 
										 double *lb, 
										 double *ub, 
										 double *lbA, 
										 double *ubA, 
										 int &nWSR, 
										 double *cputime)
{

	solver_ = new QProblem (nVar_, nConst_);


	/* Solve first QP. */
	retval_ = solver_->init( H,g,G,lb,ub,lbA,ubA, nWSR );
	
	if (retval_ != SUCCESSFUL_RETURN){

		ROS_ERROR("The qpOASES solver object could not be initialized");

	}

	return true;
}

bool mpc::optimizer::qpOASES::hotstartSolver(double *g_new, 
											double *G_new, 
											double *lb_new, 
											double *ub_new, 
											double *lbA_new, 
											double *ubA_new, 
											int &nWSR, 
											double *cputime,
											double &(*optSol))
{
	
	/* Solve second QP. */
	retval_ = solver_->hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

	if (retval_ != SUCCESSFUL_RETURN){

		ROS_ERROR("The quadratic problem was not successfully solved");

	}
	/* Get and print solution of second QP. */
	/*if (optSol.rows () != horizon_){
		ROS_ERROR("The rows of the solution matrix are not the same as the horizon");
	}
	
	if (optSol.cols () != inputs_){
		ROS_ERROR("The columns of the solution matrix are not the same as the number of inputs");
	}*/
	
	// 
	/*double * solution_ptr;
	solution_ptr = optSol.data();

	double sol_array[nVar_];	

	for (int t = 0; t < nVar_; t++) {
		sol_array[t] = *solution_ptr;
		solution_ptr++;*/	

	// Obtaining the solution
	solver_->getPrimalSolution( optSol );
	//Eigen::Map<Eigen::Matrix<double,5,1,Eigen::RowMajor> > optimalSolution(optSol,horizon_,inputs_);

	
	
	for (int i=0; i<nVar_; i++){
		std::cout <<"\noptSol["<< i <<"] = "<< optSol[i] << std::endl;
	}

	
	//std::cout<<"Optimal Solution ="<< optimalSolution << std::endl;	
	std::cout<<"objVal ="<< solver_->getObjVal() << std::endl;
	
	return true;
}
