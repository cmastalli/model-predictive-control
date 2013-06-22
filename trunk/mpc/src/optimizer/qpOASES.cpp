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


mpc::optimizer::qpOASES::qpOASES(ros::NodeHandle node_handle) : nh_opt_(node_handle)
{
	
	// Reading the parameters required for the solver
	if (nh_opt_.getParam("horizon", horizon_)){	
			ROS_INFO("Got param: %d", horizon_);		
	}
	
	if (nh_opt_.getParam("optimizer/number_constraints", nConst_)){	
		ROS_INFO("Got param: number of constraints = %d", nConst_);
	}

	if (nh_opt_.getParam("optimizer/number_variables", nVar_)){
		
		//if (nVar_ == horizon_*inputs_){
			ROS_INFO("Got param: number of variables = %d", nVar_);	// TODO perform the check of nVar = horizon x inputs in the STDMPC class!!!
		//}

		//else {
			//ROS_INFO("Number of variables != Prediction Horizon x number of Inputs --> Invalid number of variables");
		//}
	}

	ROS_INFO("qpOASES solver class successfully initialized");

}

USING_NAMESPACE_QPOASES

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
											double **optSol)
{
	
	/* Solve second QP. */
	retval_ = solver_->hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

	if (retval_ != SUCCESSFUL_RETURN){

		ROS_ERROR("The quadratic problem was not successfully solved");

	}
	
	/*double **address_array = 0;
	*address_array = *optSol;
	double *sol_array = 0;
	*sol_array = **address_array;*/
	
	


	// Obtaining the solution
	solver_->getPrimalSolution( *optSol );
	//Eigen::Map<Eigen::Matrix<double,5,1,Eigen::RowMajor> > optimalSolution(optSol,horizon_,inputs_);

	
	
	for (int i=0; i<nVar_; i++){
		std::cout <<"\noptSol["<< i <<"] = "<< **(optSol + i) << std::endl;
	}

	
	//std::cout<<"Optimal Solution ="<< optimalSolution << std::endl;	
	std::cout<<"objVal ="<< solver_->getObjVal() << std::endl;
	
	return true;
}
