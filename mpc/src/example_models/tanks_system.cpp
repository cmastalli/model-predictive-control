#include <ros/ros.h>
#include <iostream>

#include "mpc/example_models/tanks_system.h"


mpc::example_models::TanksSystem::TanksSystem()
{
	num_states_ = 2;
	num_inputs_ = 1;
	num_outputs_ = 1;
	op_point_states_ = new double[num_states_];

	time_variant_ = false;
}


bool mpc::example_models::TanksSystem::computeLinearSystem(Eigen::MatrixXd& A, Eigen::MatrixXd& B)
{
	A_ = Eigen::MatrixXd::Zero(num_states_, num_states_);
	B_ = Eigen::MatrixXd::Zero(num_states_, num_inputs_);

	// A matrix
	A_(0,0) = 0.9992;
	A_(0,1) = 0.0000;
	A_(1,0) = -0.000803;
	A_(1,1) = 1.001;

	// B matrix
	B_(0,0) = 0.002551;
	B_(1,0) = 0.0000;

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

	return true;

}

bool mpc::example_models::TanksSystem::computeLinearSystem(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double* op_states, double* op_inputs) { }

