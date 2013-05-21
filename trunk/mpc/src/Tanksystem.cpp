#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "mpc/test_models/Tanksystem.h"

void Tanksystem::Tanksystem(){
	// A matrix
			Eigen::Matrix2d Ass_<< 0.9992, 0.0000,
			-0.000803, 1.001;

			// B matrix
			Eigen::Vector2d Bss_<< 0.002551,
			0.0000;

			// C matrix
			// The real Css matrix is the transpose of this vector, the function getModelParameterC does this.
			Eigen::Vector2d Css_<< 0.0000,
			1.0000;
}

void Tanksystem::getModelParameterA(/*int current_time,*/ Eigen::MatrixXd A)
{

	if (A.rows() != Ass_.rows()) {
			std::cout<< "The number of rows of the destination matrix variable and the model matrix A is different!\n" << std::endl;
		}
	if else (A.cols() != Ass_.cols()) {
			std::cout<< "The number of columns of the destination matrix variable and the model matrix A is different!\n" << std::endl;
		}
	else {
			A = /*mpc::test_models::Tanksystem*/ Ass_;
		}
} // end of routine getModelParameterA


void Tanksystem::getModelParameterB(/*int current_time,*/ Eigen::MatrixXd B)
{

	if (B.rows() != Bss_.rows()) {
			std::cout<< "The number of rows of the destination matrix variable and the model matrix B is different!\n" << std::endl;
		}
	if else (B.cols() != Bss_.cols()) {
			std::cout<< "The number of columns of the destination matrix variable and the model matrix B is different!\n" << std::endl;
		}
	else {
			B = /*mpc::test_models::Tanksystem*/ Bss_;
		}
} // end of routine getModelParameterB


void Tanksystem::getModelParameterC(/*int current_time,*/ Eigen::MatrixXd C)
{

	if (C.rows() != Css_.rows()) {
			std::cout<< "The number of rows of the destination matrix variable and the model matrix C is different!\n" << std::endl;
		}
	if else (C.cols() != Css_.cols()) {
			std::cout<< "The number of columns of the destination matrix variable and the model matrix C is different!\n" << std::endl;
		}
	else {
			C = /*mpc::test_models::Tanksystem*/ Css_.transpose(); 
		}
} // end of routine getModelParameterC
