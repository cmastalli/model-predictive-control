#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

#include "mpc/example_models/ardrone.h"


mpc::example_models::ArDrone::ArDrone()
{
	/** All constants are defined in metric system units **/ 
	states_ = 12;
	inputs_ = 4;
	outputs_ = 12;
	op_point_states_ = new double[states_];
	op_point_input_ = new double[inputs_];
	u = Eigen::MatrixXd::Zero(4,1);

	Ct_ = 8.17e-006; // Thrust coefficient [N/(rad/s)^2]
	Cq_ = 2.17e-007; // Drag coefficient [Nm/(rad/s)^2]
	Ixx_ = 2.04e-005; // Inertia around the X axis [Kg/m^3]
	Iyy_ = 1.57e-005; // Inertia around the Y axis [Kg/m^3]
	Izz_ = 3.52e-005; // Inertia around the Z axis [Kg/m^3]
	m_ = 0.4305; // Mass of the quadrotor [Kg]
	d_ = 0.35; // Distance from the rotor to the mass center of the quadrotor [m]
	Jr_ = 2.5e-004; // Inertia of a unitary rotor (approximated as a disc) [Kg*m^2] 
	At_ = 0.0083; // Sampling time


}


void mpc::example_models::ArDrone::computeLTIModel()
{
	A_ = Eigen::MatrixXd::Zero(states_, states_);
	B_ = Eigen::MatrixXd::Zero(states_, inputs_);
	C_ = Eigen::MatrixXd::Identity(states_, states_);


	//Eigen::Map<Eigen::VectorXd> x(op_point_states_, 12, 1);
	//
	Eigen::VectorXd x(12);
	x(0) = 0.0;
	x(1) = 0.0;
	x(2) = 0.0;
	x(3) = 0.0;
	x(4) = 0.0; 
	x(5) = 0.0;
	x(6) = 0.0;
	x(7) = 0.0; 
	x(8) = 0.0;
	x(9) = 0.0; 
	x(10) = 0.0;
	x(11) = 0.0;

	Eigen::VectorXd u(4);
	
	

	
	/** Selection of the model depending on the roll and pitch angles (measured in radians) 
	if (abs(*(op_point_states_+6)) < 0.0175 && abs(*(op_point_states_+7)) < 0.0175) {
		A_(0,3) = 0.005;
		A_(1,4) = 0.005;
		A_(2,5) = 0.005;
		A_(0,7) = 3.375e-008;
		A_(0,10) = 5.635e-011;
		A_(1,6) = -3.375e-007;
		A_(1,9) = -5.625e-010;
		A_(3,7) = 1.35e-005;
		A_(3,10) = 3.375e-008;
		A_(4,6) = -1.35e-004;
		A_(4,9) = -3.375e-007;
		A_(6,9) = 0.005;
		A_(7,10) = 0.005;
		
		B_(0,0) = -9.221e-012;
		B_(0,2) = 9.221e-012;
		B_(1,1) = -7.096e-011;
		B_(1,3) = 7.096e-011;
		B_(2,0) = 2.35e-010;
		B_(2,1) = 2.35e-010;
		B_(2,2) = 2.35e-010;
		B_(2,3) = 2.35e-010;
		B_(3,0) = -7.377e-009;
		B_(3,2) = 7.377e-009;
		B_(4,1) = -5.677e-008;
		B_(4,3) = 5.677e-008;
		B_(5,0) = 9.4e-008;
		B_(5,1) = 9.4e-008;
		B_(5,2) = 9.4e-008;
		B_(5,3) = 9.4e-008;
		B_(6,1) = 1.262e-003;
		B_(6,1) = -1.262e-003;
		B_(7,0) = -1.639e-003;
		B_(7,2) = 1.639e-003;
		B_(8,0) = -5.563e-005;
		B_(8,1) = 5.563e-005;
		B_(8,2) = -5.563e-005;
		B_(8,3) = 5.563e-005;
		B_(9,1) = 0.5046;
		B_(9,3) = -0.5046;
		B_(10,0) = -0.6557;
		B_(10,2) = 0.6557;
		B_(11,0) = -0.02225;
		B_(11,1) = 0.02225;
		B_(11,2) = -0.02225;
		B_(11,3) = 0.02225;
	}
	 roll in (10,20) and pitch in (-10,10) 
	else if (*(op_point_states_+6) > 0.0175 && abs(*(op_point_states_+7)) < 0.0175) {
		A_(0,3) = 0.005;
		A_(1,4) = 0.005;
		A_(2,5) = 0.005;
		A_(0,7) = 3.213e-007;
		A_(0,8) = 1.163e-007;
		A_(0,10) = 5.694e-010;
		A_(0,11) = -1.064e-012;
		A_(1,6) = -3.213e-007;
		A_(1,9) = -5.354e-010;
		A_(1,11) = -1.831e-010;
		A_(2,6) = -1.163e-007;
		A_(2,9) = -1.938e-010;
		A_(2,11) = -6.626e-011;
		A_(3,7) = 1.285e-004;
		A_(3,8) = 4.65e-005;
		A_(3,10) = 3.416e-007;
		A_(3,11) = -6.274e-010;
		A_(4,6) = -1.285e-004;
		A_(4,9) = -3.213e-007;
		A_(4,11) = -1.099e-007;
		A_(5,6) = -4.65e-005;
		A_(5,9) = -1.163e-007;
		A_(5,11) = -3.976e-008;
		A_(6,9) = 0.005;
		A_(6,11) = 0.00171;
		A_(7,10) = 4.699e-003;
		A_(7,11) = -0.00171;
		A_(8,10) = 0.00171;
		A_(8,11) = 4.699e-003;
	
		B_(0,0) = -9.33e-011;
		B_(0,1) = -5.816e-015;
		B_(0,2) = 9.33e-011;
		B_(0,3) = -5.815e-015;
		B_(1,0) = -8.011e-011;
		B_(1,1) = -1.497e-011;
		B_(1,2) = -8.011e-011;
		B_(1,3) = -1.46e-010;
		B_(2,0) = 2.229e-010;
		B_(2,1) = 1.977e-010;
		B_(2,2) = 2.229e-010;
		B_(2,3) = 2.466e-010;
		B_(3,0) = -7.467e-008;
		B_(3,1) = -4.653e-012;
		B_(3,2) = 7.467e-008;
		B_(3,3) = -4.653e-012;
		B_(4,0) = -3.164e-008;
		B_(4,1) = -8.73e-008;
		B_(4,2) = -3.164e-008;
		B_(4,3) = 2.077e-008;
		B_(5,0) = 8.929e-008;
		B_(5,1) = 6.915e-007;
		B_(5,2) = 8.929e-008;
		B_(5,3) = 1.083e-008;
		B_(6,0) = -1.902e-005;
		B_(6,1) = 1.281e-003;
		B_(6,2) = -1.902e-005;
		B_(6,3) = -1.242e-003;
		B_(7,0) = -1.521e-003;
		B_(7,1) = -1.902e-005;
		B_(7,2) = 1.559e-003;
		B_(7,3) = -1.902e-005;
		B_(8,0) = -6.129e-004;
		B_(8,1) = 5.227e-005;
		B_(8,2) = 5.084e-004;
		B_(8,3) = 5.227e-005;
		B_(9,1) = 0.5046;
		B_(9,3) = -0.5046;
		B_(10,0) = -0.6557;
		B_(10,2) = 0.6557;
		B_(11,0) = -0.02225;
		B_(11,1) = 0.02225;
		B_(11,2) = -0.02225;
		B_(11,3) = 0.02225;
	}	
	/** roll in (-20,-10) and pitch in (-10,10) 
	else if (*(op_point_states_+6) < -0.0175 && abs(*(op_point_states_+7)) < 0.0175) {
		A_(0,3) = 0.005;
		A_(1,4) = 0.005;
		A_(2,5) = 0.005;
		A_(0,7) = 3.213e-007;
		A_(0,8) = -1.163e-007;
		A_(0,10) = 5.694e-010;
		A_(0,11) = -1.064e-012;
		A_(1,6) = -3.213e-007;
		A_(1,9) = -5.354e-010;
		A_(1,11) = -1.831e-010;
		A_(2,6) = 1.163e-007;
		A_(2,9) = 1.938e-010;
		A_(2,11) = -6.626e-011;
		A_(3,7) = 1.285e-004;
		A_(3,8) = -4.65e-005;
		A_(3,10) = 3.416e-007;
		A_(3,11) = -6.274e-010;
		A_(4,6) = -1.285e-004;
		A_(4,9) = -3.213e-007;
		A_(4,11) = 1.099e-007;
		A_(5,6) = 4.65e-005;
		A_(5,9) = 1.163e-007;
		A_(5,11) = -3.976e-008;
		A_(6,9) = 0.005;
		A_(6,11) = -0.00171;
		A_(7,10) = 4.699e-003;
		A_(7,11) = 0.00171;
		A_(8,10) = -0.00171;
		A_(8,11) = 4.699e-003;
	
		B_(0,0) = -9.33e-011;
		B_(0,1) = 5.816e-015;
		B_(0,2) = 9.33e-011;
		B_(0,3) = 5.815e-015;
		B_(1,0) = 8.011e-011;
		B_(1,1) = 1.46e-011;
		B_(1,2) = 8.011e-011;
		B_(1,3) = 1.497e-010;
		B_(2,0) = 2.229e-010;
		B_(2,1) = 2.466e-010;
		B_(2,2) = 2.229e-010;
		B_(2,3) = 1.977e-010;
		B_(3,0) = -7.467e-008;
		B_(3,1) = 4.653e-012;
		B_(3,2) = 7.467e-008;
		B_(3,3) = 4.653e-012;
		B_(4,0) = 3.164e-008;
		B_(4,1) = -2.077e-008;
		B_(4,2) = 3.164e-008;
		B_(4,3) = 8.73e-008;
		B_(5,0) = 8.929e-008;
		B_(5,1) = 1.083e-007;
		B_(5,2) = 8.929e-008;
		B_(5,3) = 6.915e-008;
		B_(6,0) = 1.902e-005;
		B_(6,1) = 1.242e-003;
		B_(6,2) = 1.902e-005;
		B_(6,3) = -1.281e-003;
		B_(7,0) = -1.559e-003;
		B_(7,1) = 1.902e-005;
		B_(7,2) = 1.521e-003;
		B_(7,3) = 1.902e-005;
		B_(8,0) = 5.084e-004;
		B_(8,1) = 5.227e-005;
		B_(8,2) = -6.129e-004;
		B_(8,3) = 5.227e-005;
		B_(9,1) = 0.5046;
		B_(9,3) = -0.5046;
		B_(10,0) = -0.6557;
		B_(10,2) = 0.6557;
		B_(11,0) = -0.02225;
		B_(11,1) = 0.02225;
		B_(11,2) = -0.02225;
		B_(11,3) = 0.02225; 
	}	
	/** roll in (-10,10) and pitch in (10,20) 
	else if (*(op_point_states_+7) > 0.0175 && abs(*(op_point_states_+6)) < 0.0175) {
		A_(0,3) = 0.005;
		A_(1,4) = 0.005;
		A_(2,5) = 0.005;
		A_(0,7) = 3.212e-007;
		A_(0,10) = 5.354e-010;
		A_(1,6) = -3.425e-007;
		A_(1,8) = 1.162e-007;
		A_(1,9) = -5.708e-010;
		A_(1,11) = 2.062e-010;
		A_(2,7) = -1.162e-007;
		A_(2,9) = 1.938e-010;
		A_(2,10) = -1.938e-010;
		A_(3,7) = 1.285e-004;
		A_(3,10) = 3.212e-007;
		A_(4,6) = -1.37e-004;
		A_(4,8) = 4.65e-005;
		A_(4,9) = -3.425e-007;
		A_(4,11) = 1.237e-007;
		A_(5,7) = -4.65e-005;
		A_(5,10) = -1.163e-007;
		A_(6,9) = 0.005;
		A_(7,10) = 0.005;
		A_(8,11) = 5.321e-003;
	
		B_(0,0) = -6.643e-012;
		B_(0,1) = 8.113e-011;
		B_(0,2) = 1.689e-010;
		B_(0,3) = 8.113e-011;
		B_(1,0) = -1.147e-012;
		B_(1,1) = -7.086e-011;
		B_(1,2) = -1.147e-012;
		B_(1,3) = 7.316e-011;
		B_(2,0) = 2.693e-010;
		B_(2,1) = 2.375e-010;
		B_(2,2) = 2.057e-010;
		B_(2,3) = 2.375e-010;
		B_(3,0) = -3.776e-008;
		B_(3,1) = 3.245e-008;
		B_(3,2) = 1.027e-007;
		B_(3,3) = 3.245e-008;
		B_(4,0) = -9.175e-010;
		B_(4,1) = -5.669e-008;
		B_(4,2) = -9.175e-010;
		B_(4,3) = 5.853e-008;
		B_(5,0) = 1.204e-007;
		B_(5,1) = 9.5e-008;
		B_(5,2) = 6.959e-008;
		B_(5,3) = 9.5e-008;
		B_(6,1) = 1.262e-003;
		B_(6,3) = -1.262e-003;
		B_(7,0) = -1.639e-003;
		B_(7,2) = 1.639e-003;
		B_(8,0) = -5.92e-005;
		B_(8,1) = 5.92e-005;
		B_(8,2) = -5.92e-005;
		B_(8,3) = 5.92e-005;
		B_(9,1) = 0.5046;
		B_(9,3) = -0.5046;
		B_(10,0) = -0.6557;
		B_(10,2) = 0.6557;
		B_(11,0) = -0.02225;
		B_(11,1) = 0.02225;
		B_(11,2) = -0.02225;
		B_(11,3) = 0.02225; 
	}
	/** The remaining case: roll in (-10,10) and pitch in (-20,-10) 
	else {			
		A_(0,3) = 0.005;
		A_(1,4) = 0.005;
		A_(2,5) = 0.005;
		A_(0,7) = 3.212e-007;
		A_(0,10) = 5.354e-010;
		A_(1,6) = -3.425e-007;
		A_(1,8) = -1.162e-007;
		A_(1,9) = -5.708e-010;
		A_(1,11) = -2.062e-010;
		A_(2,7) = 1.162e-007;
		A_(2,10) = 1.938e-010;
		A_(3,7) = 1.285e-004;
		A_(3,10) = 3.212e-007;
		A_(4,6) = -1.37e-004;
		A_(4,8) = -4.65e-005;
		A_(4,9) = -3.425e-007;
		A_(4,11) = -1.237e-007;
		A_(5,7) = 4.65e-005;
		A_(5,10) = 1.163e-007;
		A_(6,9) = 0.005;
		A_(7,10) = 0.005;
		A_(8,11) = 5.321e-003;
	
		B_(0,0) = -1.689e-010;
		B_(0,1) = -8.113e-011;
		B_(0,2) = 6.643e-012;
		B_(0,3) = -8.113e-011;
		B_(1,0) = 1.147e-012;
		B_(1,1) = -7.316e-011;
		B_(1,2) = 1.147e-012;
		B_(1,3) = 7.086e-011;
		B_(2,0) = 2.057e-010;
		B_(2,1) = 2.375e-010;
		B_(2,2) = 2.693e-010;
		B_(2,3) = 2.375e-010;
		B_(3,0) = -1.027e-007;
		B_(3,1) = -3.245e-008;
		B_(3,2) = 3.776e-008;
		B_(3,3) = -3.245e-008;
		B_(4,0) = 9.175e-010;
		B_(4,1) = -5.853e-008;
		B_(4,2) = 9.175e-010;
		B_(4,3) = 5.669e-008;
		B_(5,0) = 6.959e-008;
		B_(5,1) = 9.5e-008;
		B_(5,2) = 1.204e-007;
		B_(5,3) = 9.5e-008;
		B_(6,1) = 1.262e-003;
		B_(6,3) = -1.262e-003;
		B_(7,0) = -1.639e-003;
		B_(7,2) = 1.639e-003;
		B_(8,0) = -5.92e-005;
		B_(8,1) = 5.92e-005;
		B_(8,2) = -5.92e-005;
		B_(8,3) = 5.92e-005;
		B_(9,1) = 0.5046;
		B_(9,3) = -0.5046;
		B_(10,0) = -0.6557;
		B_(10,2) = 0.6557;
		B_(11,0) = -0.02225;
		B_(11,1) = 0.02225;
		B_(11,2) = -0.02225;
		B_(11,3) = 0.02225; 
	}**/

	// For convenience, we will define Cm as follows:  	
	double Cm = Ct_/m_;

	A_(0,3) = 1*At_;
	A_(1,4) = 1*At_;
	A_(2,5) = 1*At_;
	A_(3,6) = (Cm*( (u(0)*u(0)) + (u(1)*u(1)) + (u(2)*u(2)) + (u(3)*u(3)) ))*(sin(x(8))*cos(x(6)) - cos(x(8))*sin(x(7))*sin(x(6)))*At_;
	A_(3,7) = (Cm*( (u(0)*u(0)) + (u(1)*u(1)) + (u(2)*u(2)) + (u(3)*u(3)) ))*(cos(x(8))*cos(x(7))*cos(x(6)))*At_;
	A_(3,8) = (Cm*( (u(0)*u(0)) + (u(1)*u(1)) + (u(2)*u(2)) + (u(3)*u(3)) ))*(cos(x(8))*sin(x(6)) - sin(x(8))*sin(x(7))*cos(x(6)))*At_;
	A_(4,6) = -(Cm*( (u(0)*u(0)) + (u(1)*u(1)) + (u(2)*u(2)) + (u(3)*u(3)) ))*(sin(x(8))*sin(x(7))*sin(x(6)) + cos(x(8))*cos(x(6)))*At_;
	A_(4,7) = (Cm*( (u(0)*u(0)) + (u(1)*u(1)) + (u(2)*u(2)) + (u(3)*u(3)) ))*(sin(x(8))*cos(x(7))*cos(x(6)))*At_;
	A_(4,8) = (Cm*( (u(0)*u(0)) + (u(1)*u(1)) + (u(2)*u(2)) + (u(3)*u(3)) ))*(cos(x(8))*sin(x(7))*cos(x(6)) + sin(x(8))*sin(x(6)))*At_;
	A_(5,6) = -(Cm*( (u(0)*u(0)) + (u(1)*u(1)) + (u(2)*u(2)) + (u(3)*u(3)) ))*(cos(x(7))*sin(x(6)))*At_;
	A_(5,7) = -(Cm*( (u(0)*u(0)) + (u(1)*u(1)) + (u(2)*u(2)) + (u(3)*u(3)) ))*(sin(x(7))*cos(x(6)))*At_;
	A_(6,6) = x(10)*cos(x(6))*tan(x(7))*At_ - x(11)*sin(x(6))*tan(x(7))*At_;
	A_(6,7) = x(10)*sin(x(6))*(1/(cos(x(7))*cos(x(7))))*At_ -x(11)*sin(x(6))*(1/(cos(x(7))*cos(x(7))))*At_;
	A_(6,9) = 1*At_;
	A_(6,10) = sin(x(6))*tan(x(7))*At_;
	A_(6,11) = cos(x(6))*tan(x(7))*At_;
	A_(7,6) = -x(10)*sin(x(6))*At_ - x(11)*cos(x(6))*At_;
	A_(7,10) = cos(x(6))*At_;
	A_(7,11) = -sin(x(6))*At_;
	A_(8,6) = x(10)*cos(x(6))*(1/cos(x(7)))*At_ - x(11)*sin(x(6))*(1/cos(x(7)))*At_;
	A_(8,7) = x(10)*sin(x(6))*(1/cos(x(7)))*tan(x(7))*At_ + x(11)*cos(x(6))*(1/cos(x(7)))*tan(x(7))*At_;
	A_(8,10) = sin(x(6))/cos(x(7))*At_;
	A_(8,11) = cos(x(6))/cos(x(7))*At_;
	A_(9,10) = (Iyy_ - Izz_)*(x(11)/Ixx_)*At_ - Jr_*(u(0) + u(1) + u(2) + u(3) )*At_;
	A_(9,11) = (Iyy_ - Izz_)*(x(10)/Ixx_)*At_;
	A_(10,9) = (Izz_ - Ixx_)*(x(11)/Iyy_)*At_ + Jr_*(u(0) + u(1) + u(2) + u(3) )*At_;
	A_(10,11) = (Izz_ - Ixx_)*(x(9)/Iyy_)*At_;
	A_(11,9) = (Ixx_ - Iyy_)*(x(10)/Izz_)*At_;
	A_(11,10) = (Ixx_ - Iyy_)*(x(9)/Izz_)*At_;

	//std::cout <<"The A matrix for the desired linear operation point is:\n" << A_ << std::endl;
	
	B_(3,0) = (cos(x(8))*sin(x(7))*cos(x(6))*At_ + sin(x(8))*sin(x(6)))*2*Cm*u(0)*At_;
	B_(3,1) = (cos(x(8))*sin(x(7))*cos(x(6))*At_ + sin(x(8))*sin(x(6)))*2*Cm*u(1)*At_;
	B_(3,2) = (cos(x(8))*sin(x(7))*cos(x(6))*At_ + sin(x(8))*sin(x(6)))*2*Cm*u(2)*At_;
	B_(3,3) = (cos(x(8))*sin(x(7))*cos(x(6))*At_ + sin(x(8))*sin(x(6)))*2*Cm*u(3)*At_;		

	B_(4,0) = (sin(x(8))*sin(x(7))*cos(x(6))*At_ - cos(x(8))*sin(x(6)))*2*Cm*u(0)*At_;
	B_(4,1) = (sin(x(8))*sin(x(7))*cos(x(6))*At_ - cos(x(8))*sin(x(6)))*2*Cm*u(1)*At_;
	B_(4,2) = (sin(x(8))*sin(x(7))*cos(x(6))*At_ - cos(x(8))*sin(x(6)))*2*Cm*u(2)*At_;
	B_(4,3) = (sin(x(8))*sin(x(7))*cos(x(6))*At_ - cos(x(8))*sin(x(6)))*2*Cm*u(3)*At_;
	
	B_(5,0) = (cos(x(8))*cos(x(6)))*2*Cm*u(0)*At_;
	B_(5,1) = (cos(x(8))*cos(x(6)))*2*Cm*u(1)*At_;
	B_(5,2) = (cos(x(8))*cos(x(6)))*2*Cm*u(2)*At_;
	B_(5,3) = (cos(x(8))*cos(x(6)))*2*Cm*u(3)*At_;

	B_(9,0) = -Jr_*At_;
	B_(9,1) = (2*d_*d_*Ct_*u(1)*At_)/Ixx_ - Jr_*At_;
	B_(9,2) = -Jr_*At_;
	B_(9,3) = -(2*d_*d_*Ct_*u(3)*At_)/Ixx_ - Jr_*At_;

	B_(10,0) = -(2*d_*d_*Ct_*u(0)*At_)/Iyy_ - Jr_*At_;
	B_(10,1) = -Jr_*At_;
	B_(10,2) = (2*d_*d_*Ct_*u(2)*At_)/Iyy_ - Jr_*At_;
	B_(10,3) = -Jr_*At_;

	B_(11,0) = -(2*Cq_*u(0)*At_)/Izz_;
	B_(11,1) = (2*Cq_*u(1)*At_)/Izz_;
	B_(11,2) = -(2*Cq_*u(2)*At_)/Izz_;
	B_(11,3) = (2*Cq_*u(3)*At_)/Izz_;

	
	//std::cout <<"The B matrix for the desired linear operation point is:\n" << B_ << std::endl;

}

bool mpc::example_models::ArDrone::computeDynamicModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C)
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