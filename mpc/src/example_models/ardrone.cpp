#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

#include "mpc/example_models/ardrone.h"


mpc::example_models::ArDrone::ArDrone()
{
	/** All constants are defined in metric system units **/ 
	num_states_ = 12;
	num_inputs_ = 4;
	num_outputs_ = 12;
	op_point_states_ = new double[num_states_];
	op_point_input_ = new double[num_inputs_];

	A_ = Eigen::MatrixXd::Zero(num_states_, num_states_);
	B_ = Eigen::MatrixXd::Zero(num_states_, num_inputs_);

	time_variant_ = true;

	Ct_ = 8.17e-006;
	Cq_ = 2.17e-007;
	Ixx_ = 2.04e-003; 
	Iyy_ = 1.57e-003; 
	Izz_ = 3.52e-003; 
	m_ = 0.4305; 
	d_ = 0.35; 
	ts_ = 0.0083; 


}


bool mpc::example_models::ArDrone::computeLinearSystem(Eigen::MatrixXd &A, Eigen::MatrixXd &B, double* op_states, double* op_inputs)
{
	Eigen::Map<Eigen::VectorXd> x_bar(op_states, num_states_);
	Eigen::Map<Eigen::VectorXd> u_bar(op_inputs, num_inputs_);
	Eigen::MatrixXd U = Eigen::MatrixXd::Zero(num_inputs_, num_inputs_);
	
	double phi = x_bar(6);
	double theta = x_bar(7);
	double psi = x_bar(8);
	double p = x_bar(9);
	double q = x_bar(10);
	double r = x_bar(11);
	double U1 = Ct_ * (pow(u_bar(0),2) + pow(u_bar(1),2) + pow(u_bar(2),2) + pow(u_bar(3),2));
	
	
	A_(0,3) = ts_;
	A_(1,4) = ts_;
	A_(2,5) = ts_;
	A_(3,6) = ts_ * (sin(psi) * cos(phi) - cos(psi) * sin(theta) * sin(phi)) * U1 / m_;
	A_(3,7) = ts_ * (cos(psi) * cos(theta) * cos(phi)) * U1 / m_;
	A_(3,8) = ts_ * (cos(psi) * sin(phi) - sin(psi) * sin(theta) * cos(psi)) * U1 / m_;
	A_(4,6) = - ts_ * (sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi)) * U1 / m_;
	A_(4,7) = ts_ * (sin(psi) * cos(theta) * cos(psi)) * U1 / m_;
	A_(4,8) = ts_ * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) * U1 / m_;
	A_(5,6) = - ts_ * (cos(theta) * sin(phi)) * U1 / m_;
	A_(5,7) = - ts_ * (sin(theta) * cos(phi)) * U1 / m_;	
	A_(6,6) = ts_ * (q * cos(phi) - r * sin(phi)) * tan(theta);
	A_(6,7) = ts_ * (q * sin(phi) + r * cos(phi)) / (cos(theta) * cos(theta));
	A_(6,9) = ts_;
	A_(6,10) = ts_ * sin(phi) * tan(theta);
	A_(6,11) = ts_ * cos(phi) * tan(theta);
	A_(7,6) = - ts_ * (q * sin(phi) + r * cos(phi));
	A_(7,10) = ts_ * cos(phi);
	A_(7,11) = - ts_ * sin(phi);
	A_(8,6) = ts_ * (q * cos(phi) - r * sin(phi)) / cos(theta);
	A_(8,7) = ts_ * (q * sin(phi) + r * cos(phi)) * tan(theta) / cos(theta);
	A_(8,10) = ts_ * sin(phi) / cos(theta);
	A_(8,11) = ts_ * cos(phi) / cos(theta);
	A_(9,10) = ts_ * r * (Iyy_ - Izz_) / Ixx_;
	A_(9,11) = ts_ * q * (Iyy_ - Izz_) / Ixx_;
	A_(10,9) = ts_ * r * (Izz_ - Ixx_) / Iyy_;
	A_(10,11) = ts_ * p * (Izz_ - Ixx_) / Iyy_;
	A_(11,9) = ts_ * q * (Ixx_ - Iyy_) / Izz_;
	A_(11,10) = ts_ * p * (Ixx_ - Iyy_) / Izz_;
	
	B_(3,0) = ts_ * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) / m_;		
	B_(4,0) = ts_ * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)) / m_;	
	B_(5,0) = ts_ * cos(theta) * cos(phi) / m_;
	B_(9,1) = ts_ * d_ / Ixx_;
	B_(10,2) = ts_ * d_ / Iyy_;
	B_(11,3) = ts_ / Izz_;
	
	U(0,0) = 2 * Ct_ * u_bar(0);
	U(0,1) = 2 * Ct_ * u_bar(1);
	U(0,2) = 2 * Ct_ * u_bar(2);
	U(0,3) = 2 * Ct_ * u_bar(3);
	U(1,1) = - 2 * Ct_ * u_bar(1);
	U(1,3) = 2 * Ct_ * u_bar(3);
	U(2,0) = 2 * Ct_ * u_bar(0);
	U(2,2) = - 2 * Ct_ * u_bar(2);
	U(3,0) = - 2 * Cq_ * u_bar(0);
	U(3,1) = 2 * Cq_ * u_bar(1);
	U(3,2) = - 2 * Cq_ * u_bar(2);
	U(3,3) = 2 * Cq_ * u_bar(3);
	
	B_ = B_ * U; 


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

bool mpc::example_models::ArDrone::computeLinearSystem(Eigen::MatrixXd& A, Eigen::MatrixXd& B) { }


