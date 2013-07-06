#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <mpc/optimizer/qpOASES.h>
#include <mpc/model/model.h>
#include <mpc/optimizer/optimizer.h>


USING_NAMESPACE_QPOASES


mpc::optimizer::qpOASES::qpOASES(ros::NodeHandle node_handle) : nh_(node_handle)
{
	nVar_ = 0;
	nConst_ = 0;
	horizon_ = 0;
}

bool mpc::optimizer::qpOASES::init()
{
	// reading the parameters required for the solver
	if (nh_.getParam("optimizer/number_constraints", nConst_)) {
		ROS_INFO("Got param: number of constraints = %d", nConst_);
	}
	
	nh_.param<int>("optimizer/working_set_recalculations", nWSR_, 10);
	ROS_INFO("Got param: number of working set recalculations = %d", nWSR_);
	
	if (nVar_ == 0 || nConst_ == 0 || horizon_ == 0)
		return false;
	
	qpOASES_initialized_ = false;
	solver_ = new SQProblem(nVar_, nConst_, HST_SEMIDEF);
/*	Options myOptions;
	myOptions.setToReliable();
	myOptions.printLevel = PL_LOW;
	solver_->setOptions(myOptions);*/
	
	optimal_solution_ = new double[nVar_];
	
	
	ROS_INFO("qpOASES solver class successfully initialized.");
	return true;
}


bool mpc::optimizer::qpOASES::computeOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbA, double *ubA, double cputime)
{
	// solve first QP.
	if (!qpOASES_initialized_) {
		retval_ = solver_->init(H, g, G, lb, ub, lbA, ubA, nWSR_);
		if (retval_ == SUCCESSFUL_RETURN) {
			ROS_INFO("qpOASES problem successfully initialized");
			qpOASES_initialized_ = true;
		}
	}
	else {
		retval_ = solver_->hotstart(g, lb, ub, lbA, ubA, nWSR_, &cputime);

		if (retval_ == SUCCESSFUL_RETURN) {
			solver_->getPrimalSolution(optimal_solution_);
			return true;
		}
		else if (retval_ == RET_MAX_NWSR_REACHED) {
			ROS_WARN("The QP couldn't solve because the maximun number of WSR was reached.");
			return false;
		}
		else { 
			ROS_WARN("The QP couldn't find the solution.");
			return false;
		}
	}
	
	return true;
}

double* mpc::optimizer::qpOASES::getOptimalSolution()
{
		return optimal_solution_;
}




