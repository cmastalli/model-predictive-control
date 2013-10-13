#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

void computeLTIModel(double* point_states_ptr, double* input_states_ptr)
{
	Eigen::MatrixXd A_ = Eigen::MatrixXd::Identity(12, 12);
	Eigen::MatrixXd B_ = Eigen::MatrixXd::Zero(12, 4);

	Eigen::Map<Eigen::VectorXd> x(point_states_ptr, 12);
	Eigen::Map<Eigen::VectorXd> u(input_states_ptr, 4);
	
	// A matrix
	double Ct_ = 8.17e-006; // Thrust coefficient [N/(rad/s)^2]
	double Cq_ = 2.17e-007; // Drag coefficient [Nm/(rad/s)^2]
	double Ixx_ = 2.04e-005; // Inertia around the X axis [Kg/m^3]
	double Iyy_ = 1.57e-005; // Inertia around the Y axis [Kg/m^3]
	double Izz_ = 3.52e-005; // Inertia around the Z axis [Kg/m^3]
	double m_ = 0.4305; // Mass of the quadrotor [Kg]
	double d = 0.35;
	double Jr_ = 2.5e-004; // Inertia of a unitary rotor (approximated as a disc) [Kg*m^2] 
	

	/** For convenience, we will define Cm as follows:  	
	double Cm = Ct_/m_;

	A_(3,6) = (Cm*( u(0)^2 + u(1)^2 + u(2)^2 + u(3)^2 ))*(sin(x(8))*cos(x(6)) - cos(x(8))*sin(x(7))*sin(x(6)));
	A_(3,7) = (Cm*( u(0)^2 + u(1)^2 + u(2)^2 + u(3)^2 ))*(cos(x(8))*cos(x(7))*cos(x(6)));
	A_(3,8) = (Cm*( u(0)^2 + u(1)^2 + u(2)^2 + u(3)^2 ))*(cos(x(8))*sin(x(6)) - sin(x(8))*sin(x(7))*cos(x(6)));
	A_(4,6) = -(Cm*( u(0)^2 + u(1)^2 + u(2)^2 + u(3)^2 ))*(sin(x(8))*sin(x(7))*sin(x(6)) + cos(x(8))*cos(x(6)));
	A_(4,7) = (Cm*( u(0)^2 + u(1)^2 + u(2)^2 + u(3)^2 ))*(sin(x(8))*cos(x(7))*cos(x(6)));
	A_(4,8) = (Cm*( u(0)^2 + u(1)^2 + u(2)^2 + u(3)^2 ))*(cos(x(8))*sin(x(7))*cos(x(6)) + sin(x(8))*sin(x(6)));
	A_(5,6) = -(Cm*( u(0)^2 + u(1)^2 + u(2)^2 + u(3)^2 ))*(cos(x(7))*sin(x(6)));
	A_(5,7) = -(Cm*( u(0)^2 + u(1)^2 + u(2)^2 + u(3)^2 ))*(sin(x(7))*cos(x(6)));
	A_(6,6) = x(10)*cos(x(6))*tan(x(7)) - x(11)*sin(x(6))*tan(x(7));
	A_(6,7) = x(10)*sin(x(6))*(1/(cos(x(7))*cos(x(7)))) -x(11)*sin(x(6))*(1/(cos(x(7))*cos(x(7))));
	A_(6,9) = 1;
	A_(6,10) = sin(x(6))*tan(x(7));
	A_(6,11) = cos(x(6))*tan(x(7));
	A_(7,6) = -x(10)*sin(x(6)) - x(11)*cos(x(6));
	A_(7,10) = cos(x(6));
	A_(7,11) = -sin(x(6));
	A_(8,6) = x(10)*cos(x(6))*(1/cos(x(7))) - x(11)*sin(x(6))*(1/cos(x(7)));
	A_(8,7) = x(10)*sin(x(6))*(1/cos(x(7)))*tan(x(7)) + x(11)*cos(x(6))*(1/cos(x(7)))*tan(x(7));
	A_(8,10) = sin(x(6))/cos(x(7));
	A_(8,11) = cos(x(6))/cos(x(7));
	A_(9,10) = (Iyy_ - Izz_)*(x(11)/Ixx_) - Jr_*(u(0) + u(1) + u(2) + u(3) );
	A_(9,11) = (Iyy_ - Izz_)*(x(10)/Ixx_);
	A_(10,9) = (Izz_ - Ixx_)*(x(11)/Iyy_) + Jr_*(u(0) + u(1) + u(2) + u(3) );
	A_(10,11) = (Izz_ - Ixx_)*(x(9)/Iyy_);
	A_(11,9) = (Ixx_ - Iyy_)*(x(10)/Izz_);
	A_(11,10) = (Ixx_ - Iyy_)*(x(9)/Izz_);

	std::cout <<"The A matrix for the desired linear operation point is:\n" << A_ << std::endl;
	
	B_(3,0) = (cos(x(8))*sin(x(7))*cos(x(6)) + sin(x(8))*sin(x(6)))*2*Cm*u(0);
	B_(3,1) = (cos(x(8))*sin(x(7))*cos(x(6)) + sin(x(8))*sin(x(6)))*2*Cm*u(1);
	B_(3,2) = (cos(x(8))*sin(x(7))*cos(x(6)) + sin(x(8))*sin(x(6)))*2*Cm*u(2);
	B_(3,3) = (cos(x(8))*sin(x(7))*cos(x(6)) + sin(x(8))*sin(x(6)))*2*Cm*u(3);		

	B_(4,0) = (sin(x(8))*sin(x(7))*cos(x(6)) - cos(x(8))*sin(x(6)))*2*Cm*u(0);
	B_(4,1) = (sin(x(8))*sin(x(7))*cos(x(6)) - cos(x(8))*sin(x(6)))*2*Cm*u(1);
	B_(4,2) = (sin(x(8))*sin(x(7))*cos(x(6)) - cos(x(8))*sin(x(6)))*2*Cm*u(2);
	B_(4,3) = (sin(x(8))*sin(x(7))*cos(x(6)) - cos(x(8))*sin(x(6)))*2*Cm*u(3);
	
	B_(5,0) = (cos(x(8))*cos(x(6)))*2*Cm*u(0);
	B_(5,1) = (cos(x(8))*cos(x(6)))*2*Cm*u(1);
	B_(5,2) = (cos(x(8))*cos(x(6)))*2*Cm*u(2);
	B_(5,3) = (cos(x(8))*cos(x(6)))*2*Cm*u(3);

	B_(9,0) = -Jr_;
	B_(9,1) = (2*d^2*Ct_*u(1))/Ixx_ - Jr_;
	B_(9,2) = -Jr_;
	B_(9,3) = -(2*d^2*Ct_*u(3))/Ixx_ - Jr_;

	B_(10,0) = -(2*d^2*Ct_*u(0))/Iyy_ - Jr_;
	B_(10,1) = -Jr_;
	B_(10,2) = (2*d^2*Ct_*u(2))/Iyy_ - Jr_;
	B_(10,3) = -Jr_;

	B_(11,0) = -(2*Cq_*u(0))/Izz_;
	B_(11,1) = (2*Cq_*u(1))/Izz_;
	B_(11,2) = -(2*Cq_*u(2))/Izz_;
	B_(11,3) = (2*Cq_*u(3))/Izz_;

	
	std::cout <<"The B matrix for the desired linear operation point is:\n" << B_ << std::endl;
	**/
}

int main()
{

	/** Desired operation point **/
	double state_point1[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	double input_point1[4] = {360, 360, 360, 360};

		
	computeLTIModel(state_point1, input_point1);
 
	return 0;


}
