#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

#include "mpc/example_models/ardrone_hovering.h"


mpc::example_models::ArDroneHovering::ArDroneHovering()
{
	/** All constants are defined in metric system units **/ 
	num_states_ = 10;
	num_inputs_ = 4;
	num_outputs_ = 10;
	op_point_states_ = new double[num_states_];
	op_point_input_ = new double[num_inputs_];

	A_ = Eigen::MatrixXd::Zero(num_states_, num_states_);
	B_ = Eigen::MatrixXd::Zero(num_states_, num_inputs_);

	time_variant_ = false;

	ts_ = 0.0083;
	c1_ = 10.324;//0.58;
	c2_ = 0.58;//17.8;
	c3_ = 350.;//10.;
	c4_ = 10.;//35.;
	c5_ = 250.;//10.;
	c6_ = 10.;//25.;
	c7_ = 1.4;//1.4;
	c8_ = 1.4;//1.0;
}


void mpc::example_models::ArDroneHovering::setLinearizationPoints(double* op_states)
{
	for (int i = 0; i < num_states_; i++) {
		op_point_states_[i] = op_states[i];
	}

/*	Eigen::MatrixXd M = Eigen::MatrixXd::Zero(num_inputs_, num_inputs_);
	M << 1., 1., 1., 1., 0., -1., 0., 1., 1., 0., -1., 0., -1., 1., -1., 1.;
	Eigen::VectorXd f_bar = Eigen::MatrixXd::Zero(num_inputs_, 1);

	Eigen::Map<Eigen::VectorXd> u_bar(op_point_input_, num_inputs_);


	double phi = op_point_states_[6];
	double theta = op_point_states_[7];
	double p = op_point_states_[9];
	double q = op_point_states_[10];
	double r = op_point_states_[11];

	f_bar(0) = g_ * m_ / (Ct_ * cos(phi) * cos(theta));
	f_bar(1) = (Izz_ - Iyy_) * q * r / (Ct_ * d_);
	f_bar(2) = (Izz_ - Ixx_) * p * r / (Ct_ * d_);
	f_bar(3) = (Iyy_ - Ixx_) * p * q / Cq_;
	u_bar = M.inverse() * f_bar;
	u_bar = u_bar.cwiseSqrt();*/
}


bool mpc::example_models::ArDroneHovering::computeLinearSystem(Eigen::MatrixXd &A, Eigen::MatrixXd &B)
{
	Eigen::Map<Eigen::VectorXd> x_bar(op_point_states_, num_states_);
	Eigen::Map<Eigen::VectorXd> u_bar(op_point_input_, num_inputs_);
	Eigen::MatrixXd U = Eigen::MatrixXd::Zero(num_inputs_, num_inputs_);
	
	double phi = x_bar(6);
	double theta = x_bar(7);
	double psi = x_bar(8);
	
	A_(0,0) = 1.;
	A_(0,3) = ts_;
	A_(1,1) = 1.;
	A_(1,4) = ts_;
	A_(2,2) = 1.;
	A_(2,5) = ts_;
	A_(3,3) = 1. - ts_ * c2_;
	A_(3,6) = ts_ * c1_ * cos(psi) * cos(phi) * cos(theta);
	A_(3,7) = - ts_ * c1_ * (cos(psi) * sin(phi) * sin(theta) + sin(psi) * cos(theta));
	A_(3,8) = - ts_ * c1_ * (sin(psi) * sin(phi) * cos(theta) + cos(psi) * sin(theta));
	A_(4,4) = 1. - ts_ * c2_;
	A_(4,6) = - ts_ * c1_ * sin(psi) * cos(phi) * cos(theta);
	A_(4,7) = ts_ * c1_ * (sin(psi) * sin(phi) * sin(theta) - cos(psi) * cos(theta));
	A_(4,8) = ts_ * c1_ * (-cos(psi) * sin(phi) * cos(theta) + sin(psi) * sin(theta));
	A_(5,5) = 1. - ts_ * c8_;
	A_(6,6) = 1. - ts_ * c4_;
	A_(7,7) = 1. - ts_ * c4_;	
	A_(8,8) = 1.;
	A_(8,9) = - ts_ * c6_;
	A_(9,9) = 1.;
	
	B_(5,2) = ts_ * c7_;
	B_(6,0) = ts_ * c3_;
	B_(7,1) = ts_ * c3_;
	B_(9,3) = ts_ * c5_;

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
