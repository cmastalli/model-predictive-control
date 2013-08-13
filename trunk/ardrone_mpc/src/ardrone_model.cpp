#include <ros/ros.h>
#include <iostream>

#include <ardrone_mpc/ardrone_model.h>


ardrone_mpc::ArDroneModel::ArDroneModel()
{
	states_ = 2;
	inputs_ = 1;
	outputs_ = 1;
}


void ardrone_mpc::ArDroneModel::computeLTIModel()
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

bool ardrone_mpc::ArDroneModel::computeDynamicModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C)
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

