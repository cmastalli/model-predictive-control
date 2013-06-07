#include <ros/ros.h>
#include <iostream>

#include "mpc/test_models/Tanksystem.h"


mpc::test_models::Tanksystem::Tanksystem(ros::NodeHandle node) : n_(node)
{
	
	int states, inputs, outputs;
	if (n_.getParam("states", states))
	{
		ROS_INFO("Got param: states");
	}

	if (n_.getParam("inputs", inputs))
	{
		ROS_INFO("Got param: inputs");
	}	
	
	if (n_.getParam("outputs", outputs))
	{
		ROS_INFO("Got param: outputs");
	}

	Ass_ = Eigen::MatrixXd::Zero(states,states);
	Bss_ = Eigen::MatrixXd::Zero(states,inputs);
	Css_ = Eigen::MatrixXd::Zero(outputs,states);

	// A matrix
	Ass_(0,0) = 0.9992;
	Ass_(0,1) = 0.0000;
	Ass_(1,0) = -0.000803;
	Ass_(1,1) = 1.001;

	// B matrix
	Bss_(0,0) = 0.002551;
	Bss_(1,0) = 0.0000;
			
	// C matrix
	Css_(0,0) = 0.0000;
	Css_(0,1) = 1.0000;

	ROS_INFO("Tanksystem class successfully initialized");
}

void mpc::test_models::Tanksystem::getModelParameterA(Eigen::MatrixXd& A)
{
	if (A.rows() != Ass_.rows())
		std::cout<< "The number of rows of the destination matrix variable and the model matrix A is different!\n" << std::endl;
	else if (A.cols() != Ass_.cols())
		std::cout<< "The number of columns of the destination matrix variable and the model matrix A is different!\n" << std::endl;
	else
		A = Ass_;
		
} // end of routine getModelParameterA


void mpc::test_models::Tanksystem::getModelParameterB(Eigen::MatrixXd& B)
{
	if (B.rows() != Bss_.rows())
		std::cout<< "The number of rows of the destination matrix variable and the model matrix B is different!\n" << std::endl;
	else if (B.cols() != Bss_.cols())
		std::cout<< "The number of columns of the destination matrix variable and the model matrix B is different!\n" << std::endl;
	else
		B = Bss_;

} // end of routine getModelParameterB


void mpc::test_models::Tanksystem::getModelParameterC(Eigen::MatrixXd& C)
{
	if (C.rows() != Css_.rows())
		std::cout<< "The number of rows of the destination matrix variable and the model matrix C is different!\n" << std::endl;
	else if (C.cols() != Css_.cols())
		std::cout<< "The number of columns of the destination matrix variable and the model matrix C is different!\n" << std::endl;
	else
		C = Css_; 

} // end of routine getModelParameterC
