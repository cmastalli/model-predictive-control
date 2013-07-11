#include <ros/ros.h>
#include <mpc/example_models/tanks_system_simulator.h>
#include <Eigen/Dense>



mpc::example_models::TanksSystemSimulator::TanksSystemSimulator()
{
	beta_ = 3.96;
	cf_ = 0.178;
	At_ = 15.52;
	g_ = 981.;
	new_state_ = new double[2];
}


double* mpc::example_models::TanksSystemSimulator::simulatePlant(double *current_state, double *current_input, double sampling_time)
{
	ROS_ASSERT(sizeof(new_state_) == sizeof(current_state));
	
	// Solve the difference equations recursively
	double input = *current_input;
	Eigen::Map<Eigen::VectorXd> x_current(current_state, 2, 1);
	
	new_state_[0] = x_current(0) + sampling_time * (beta_ * input - cf_ * sqrt(2 * g_ * x_current(0))) / At_;
	new_state_[1] = x_current(1) + sampling_time * cf_ * (sqrt(2 * g_ * x_current(0)) - sqrt(2 * g_ * x_current(1))) / At_;
//	new_state_[0] = 0.9992 * x_current(0) + 0.002551 * input;
//	new_state_[1] = -0.000803 * x_current(0) + 1.001 * x_current(1);
	
	
	return new_state_;
}
