#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "mpc/test_models/Tanksystem.h"

mpc::test_models::Tanksystem::Tanksystem(){
			// A matrix
			Ass_(0,0) = 0.9992;
			Ass_(0,1) = 0.0000;
			Ass_(1,0) = -0.000803;
			Ass_(1,1) = 1.001;


			// B matrix
			Bss_(0) = 0.002551;
			Bss_(1) = 0.0000;
			
			// C matrix
			// The real Css matrix is the transpose of this vector, the function getModelParameterC does this.

			Css_(0) = 0.0000;
			Css_(0) = 1.0000;
}

void mpc::test_models::Tanksystem::getModelParameterA(Eigen::MatrixXd& A)
{

	if (A.rows() != Ass_.rows()) {
			std::cout<< "The number of rows of the destination matrix variable and the model matrix A is different!\n" << std::endl;
		}
	else if (A.cols() != Ass_.cols()) {
			std::cout<< "The number of columns of the destination matrix variable and the model matrix A is different!\n" << std::endl;
		}
	else {
			A = /*mpc::test_models::Tanksystem*/ Ass_;
		}
} // end of routine getModelParameterA


void mpc::test_models::Tanksystem::getModelParameterB(Eigen::MatrixXd& B)
{

	if (B.rows() != Bss_.rows()) {
			std::cout<< "The number of rows of the destination matrix variable and the model matrix B is different!\n" << std::endl;
		}
	else if (B.cols() != Bss_.cols()) {
			std::cout<< "The number of columns of the destination matrix variable and the model matrix B is different!\n" << std::endl;
		}
	else {
			B = /*mpc::test_models::Tanksystem*/ Bss_;
		}
} // end of routine getModelParameterB


void mpc::test_models::Tanksystem::getModelParameterC(Eigen::MatrixXd& C)
{

	if (C.rows() != Css_.rows()) {
			std::cout<< "The number of rows of the destination matrix variable and the model matrix C is different!\n" << std::endl;
		}
	else if (C.cols() != Css_.cols()) {
			std::cout<< "The number of columns of the destination matrix variable and the model matrix C is different!\n" << std::endl;
		}
	else {
			C = /*mpc::test_models::Tanksystem*/ Css_.transpose(); 
		}
} // end of routine getModelParameterC
