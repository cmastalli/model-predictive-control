#include <ros/ros.h>
#include <mpc/example_models/ardrone_simulator.h>
#include <Eigen/Dense>



mpc::example_models::ArDroneSimulator::ArDroneSimulator()
{
	g_ = 9.81;
	Ixx_ = 2.04e-003; 
	Iyy_ = 1.57e-003; 
	Izz_ = 3.52e-003; 
	m_ = 0.4305; 
	d_ = 0.35; 
	Ct_ = 8.17e-006;
	Cq_ = 2.17e-007;

	number_of_states_ = 12;
	number_of_inputs_ = 4;	
	new_state_ = new double[number_of_states_];
}


double* mpc::example_models::ArDroneSimulator::simulatePlant(double *current_state, double *current_input, double sampling_time)
{
	//ROS_ASSERT(sizeof(current_state) == number_of_states_);
	//ROS_ASSERT(sizeof(new_state_) == sizeof(current_state));
	//ROS_ASSERT(sizeof(current_input) == number_of_inputs_);
	double ts_;
	ts_ = sampling_time;

		// Map into Eigen objects for easier manipulation	
	Eigen::Map<Eigen::VectorXd> x_current(current_state, 12, 1);
	Eigen::Map<Eigen::VectorXd> u_current(current_input, 4, 1);	
	Eigen::VectorXd x_new = Eigen::MatrixXd::Zero(number_of_states_, 1);

	double phi = x_current(6);
	double theta = x_current(7);
	double psi = x_current(8);
	double p = x_current(9);
	double q = x_current(10);
	double r = x_current(11);
	double U1 = Ct_ * (pow(u_current(0),2) + pow(u_current(1),2) + pow(u_current(2),2) + pow(u_current(3),2));
	double U2 = Ct_ * (- pow(u_current(1),2) + pow(u_current(3),2));
	double U3 = Ct_ * (pow(u_current(0),2) - pow(u_current(2),2));
	double U4 = Cq_ * (-pow(u_current(0),2) + pow(u_current(1),2) - pow(u_current(2),2) + pow(u_current(3),2));
	
	
	// Solve the difference equations recursively
	x_new(0) = x_current(0) + ts_ * x_current(3);
	x_new(1) = x_current(1) + ts_ * x_current(4);
	x_new(2) = x_current(2) + ts_ * x_current(5);
	x_new(3) = x_current(3) + (ts_ / m_) * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) * U1;
	x_new(4) = x_current(4) + (ts_ / m_) * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)) * U1;
	x_new(5) = x_current(5) + (ts_ / m_) * (-m_ * g_ + cos(theta) * cos(phi) * U1);
	x_new(6) = x_current(6) + ts_ * (p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta));
	x_new(7) = x_current(7) + ts_ * (q * cos(phi) - r * sin(phi));
	x_new(8) = x_current(8) + ts_ * (q * sin(phi) + r * cos(phi)) / cos(theta);
	x_new(9) = x_current(9) + ts_ * ((Iyy_ - Izz_) * q * r / Ixx_ + (d_ / Ixx_) * U2);
	x_new(10) = x_current(10) + ts_* ((Izz_ - Ixx_) * p * r / Iyy_ + (d_ / Iyy_) * U3);
	x_new(11) = x_current(11) + ts_* ((Ixx_ - Iyy_) * p * q / Izz_ + (1 / Izz_) * U4);
	
	
	x_current = x_new;
	return current_state;
}
