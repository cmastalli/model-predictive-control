#include <ros/ros.h>
#include <iostream>

#include "mpc/example_models/tanks_system.h"


mpc::example_models::TanksSystem::TanksSystem()
{
	states_ = 2;
	inputs_ = 1;
	outputs_ = 1;
}


void mpc::example_models::TanksSystem::computeLTIModel()
{
	A_ = Eigen::MatrixXd::Zero(states_, states_);
	B_ = Eigen::MatrixXd::Zero(states_, inputs_);
	C_ = Eigen::MatrixXd::Zero(outputs_, states_);

	// A matrix
	A_(0,0) = 0.9992;
	A_(0,1) = 0.0000;
	A_(1,0) = -0.000803;
	A_(1,1) = 1.001;

	// B matrix
	B_(0,0) = 0.002551;
	B_(1,0) = 0.0000;
			
	// C matrix
	C_(0,0) = 0.0000;
	C_(0,1) = 1.0000;
}

bool mpc::example_models::TanksSystem::computeDynamicModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C)
{
	computeLTIModel();

	if (A.rows() != A_.rows()) {
		ROS_ERROR("The number of rows of the destination matrix variable and the model matrix A is different!");
		return false;
	}
	else if (A.cols() != A_.cols()) {
		ROS_ERROR("The number of columns of the destination matrix variable and the model matrix A is different!");
		return false;
	}
	else
		A = A_;


	if (B.rows() != B_.rows()) {
		ROS_ERROR("The number of rows of the destination matrix variable and the model matrix B is different!");
		return false;
	}
	else if (B.cols() != B_.cols()) {
		ROS_ERROR("The number of columns of the destination matrix variable and the model matrix B is different!");
		return false;
	}
	else
		B = B_;

	if (C.rows() != C_.rows()) {
		ROS_ERROR("The number of rows of the destination matrix variable and the model matrix C is different!");
		return false;
	}
	else if (C.cols() != C_.cols()) {
		ROS_ERROR("The number of columns of the destination matrix variable and the model matrix C is different!");
		return false;
	}
	else
		C = C_;


	return true;
}

