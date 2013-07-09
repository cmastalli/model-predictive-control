#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

//#include <mpc/model/simulator.h>
#include <mpc/example_models/tanks_system_simulator.h>
//#include <mpc/model/model.h>

mpc::example_models::TanksSystemSimulator::TanksSystemSimulator()
{
	param1_ = 0.2552;
	param2_ = 0.0508; 
	state_vect_ = new double[2];
}


double* mpc::example_models::TanksSystemSimulator::simulatePlant(double *state_vect, double *input_vect, double samplingTime	)
{

// Assignment of the readed variables

	//double Hone_k = *state_vect;	// Level of the first tank in the current time step
	//double Htwo_k = *(state_vect + 1);	// Level of the second tank in the current time step

	ROS_ASSERT(sizeof(state_vect_) == sizeof(state_vect));
	state_vect_ = state_vect;

	// Solve the difference equations recursively

	*state_vect_ = *state_vect_ + samplingTime*param1_*(*input_vect) - samplingTime*param2_*sqrt(*state_vect_);

	*(state_vect_ + 1) = *(state_vect_ + 1) + samplingTime*param2_*sqrt(*(state_vect_)) - samplingTime*param2_*sqrt(*(state_vect_ + 1));


	return state_vect_; 

}
