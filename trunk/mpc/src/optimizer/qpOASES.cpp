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
			ROS_INFO("Got param: horizon = %d", horizon_);		
	}
	
	if (nh_opt_.getParam("optimizer/number_constraints", nConst_)){	
		ROS_INFO("Got param: number of constraints = %d", nConst_);
	}
	
	nh_opt_.param<int>("optimizer/working_set_recalculation", nWSR_, 10);

	if (nh_opt_.getParam("optimizer/number_variables", nVar_)){
		
		//if (nVar_ == horizon_*inputs_){
			ROS_INFO("Got param: number of variables = %d", nVar_);	// TODO perform the check of nVar = horizon x inputs in the STDMPC class!!!
		//}

		//else {
			//ROS_INFO("Number of variables != Prediction Horizon x number of Inputs --> Invalid number of variables");
		//}
	}
	
	initOnce_ = false;
	solver_ = new SQProblem (nVar_, nConst_);
	optimalSol_ = new double[nVar_];

	ROS_INFO("qpOASES solver class successfully initialized");

}

USING_NAMESPACE_QPOASES

bool mpc::optimizer::qpOASES::computeOpt(double *H, 
										 double *g, 
										 double *G, 
										 double *lb, 
										 double *ub, 
										 double *lbA, 
										 double *ubA,  
										 double *cputime)
{

	/* Solve first QP. */
	if (!initOnce_){
		
		retval_ = solver_->init( H,g,G,lb,ub,lbA,ubA, nWSR_ );

		if (retval_ == SUCCESSFUL_RETURN){
			ROS_INFO("qpOASES problem successfully initiated");
			initOnce_ = true;
		}
	}

	else {
		
		retval_ = solver_->hotstart( g,lb,ub,lbA,ubA, nWSR_, cputime );

		if (retval_ == SUCCESSFUL_RETURN){

			ROS_INFO("The quadratic problem was successfully solved");
			solver_->getPrimalSolution( optimalSol_ );

			// Solution printing
			//std::cout <<"\noptimal Solution = "<< optimalSol_[0] << std::endl;
			std::cout<<"objVal ="<< solver_->getObjVal() << std::endl;
			return true;
		}

		else if (retval_ == RET_MAX_NWSR_REACHED){
			ROS_INFO("1");
			return false;
		}
		else { 
			ROS_INFO("2");
			return false;
		}

	}

		
	
}

double* mpc::optimizer::qpOASES::getOptimalSolution()
{
	if (retval_ == SUCCESSFUL_RETURN){
		return optimalSol_;
	}
	else{
		return NULL;
		ROS_ERROR("An optimal solution was not found");
	}
}




