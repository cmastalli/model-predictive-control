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

	number_of_states_ = 12;
	number_of_inputs_ = 4;	
	new_state_ = new double[number_of_states_];
}


double* mpc::example_models::ArDroneSimulator::simulatePlant(double *current_state, double *current_input, double sampling_time)
{
	ROS_ASSERT(sizeof(current_state) == number_of_states_);
	ROS_ASSERT(sizeof(new_state_) == sizeof(current_state));
	ROS_ASSERT(sizeof(current_input) == number_of_inputs_);

	// Map into Eigen objects for easier manipulation	
	
	Eigen::Map<Eigen::VectorXd> x_current(current_state, 12, 1);
	Eigen::Map<Eigen::VectorXd> u_current(current_input, 4, 1);	

	// Solve the difference equations recursively

	new_state_[0] = x_current(0) + sampling_time*x_current(3);

	new_state_[1] = x_current(1) + sampling_time*x_current(4);

	new_state_[2] = x_current(2) + sampling_time*x_current(5);

	new_state_[3] = x_current(3) + (sampling_time/m_)*(cos(x_current(8))*sin(x_current(7))*cos(x_current(6)) + sin(x_current(8))*sin(x_current(6)))*u_current(0);

	new_state_[4] = x_current(4) + (sampling_time/m_)*(sin(x_current(8))*sin(x_current(7))*cos(x_current(6)) - cos(x_current(8))*sin(x_current(6)))*u_current(0);

	new_state_[5] = x_current(5) + (sampling_time/m_)*(cos(x_current(8))*sin(x_current(7))*cos(x_current(6)) + sin(x_current(8))*sin(x_current(6)))*u_current(0);

	new_state_[6] = x_current(6) + sampling_time*(x_current(9) + x_current(10)*sin(x_current(6))*tan(x_current(7)) + x_current(11)*cos(x_current(6))*tan(x_current(8)));

	new_state_[7] = x_current(7) + sampling_time*(x_current(10)*cos(x_current(6)) - x_current(11)*sin(x_current(6)));

	new_state_[8] = x_current(8) + (sampling_time/cos(x_current(7)))*(x_current(10)*sin(x_current(6)) + x_current(11)*cos(x_current(6)));

	new_state_[9] = x_current(9) + sampling_time*((Iyy_ - Izz_)/((x_current(10)*x_current(11))/Ixx_) + (d_/Ixx_)*u_current(1));

	new_state_[10] = x_current(10) + sampling_time*((Izz_ - Ixx_)/((x_current(9)*x_current(11))/Iyy_) + (d_/Iyy_)*u_current(2));

	new_state_[11] = x_current(11) + sampling_time*((Ixx_ - Iyy_)/((x_current(9)*x_current(10))/Izz_) + (1/Izz_)*u_current(3));

	
	return new_state_;
}
