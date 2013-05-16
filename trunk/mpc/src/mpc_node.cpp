#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ctime>

#include <model.h>
#include <optimizer.h>
#include <test_models.h>


int main ();
{
/*
Global variable definition
*/

int n_, np_, p_;
Eigen::MatrixXd H_;
Eigen::MatrixXd F_;

/*
Instantiation of the required class objects
*/

mpc::test_models::Tanksystem();
mpc::optimizer::Solver_qpOASES();

while (1){

	/*
	Function callings to perform the neccesary operations
	*/

	// Reading sensor topic
	ros::init(argv, argv, "mpc_node");
	ros::NodeHandle read;
	

	// Performing the prediction in the horizon


	// Precomputing solver parameters


	// Compute optimal solution from solver


	// Write the current time optimal solution to the actuator system


	// Send the rest of optimal inputs to the model
	
	





}
