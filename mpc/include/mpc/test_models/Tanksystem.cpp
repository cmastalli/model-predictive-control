#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "Tanksystem.h"

void getModelParameterA(int current_time, Eigen::MatrixXd A)
{

	if (A.rows() != Ass.rows()) {
			std::cout<< "The number of rows of the destination matrix variable and the model matrix A is different!\n" << std::endl;
		}
	if else (A.cols() != Ass.cols()) {
			std::cout<< "The number of columns of the destination matrix variable and the model matrix A is different!\n" << std::endl;
		}
	else {
			A = /*mpc::test_models::Tanksystem*/ Ass;
		}
} // end of routine getModelParameterA


void getModelParameterB(int current_time, Eigen::MatrixXd B)
{

	if (B.rows() != Bss.rows()) {
			std::cout<< "The number of rows of the destination matrix variable and the model matrix B is different!\n" << std::endl;
		}
	if else (B.cols() != Bss.cols()) {
			std::cout<< "The number of columns of the destination matrix variable and the model matrix B is different!\n" << std::endl;
		}
	else {
			B = /*mpc::test_models::Tanksystem*/ Bss;
		}
} // end of routine getModelParameterB


void getModelParameterC(int current_time, Eigen::MatrixXd C)
{

	if (C.rows() != Css.rows()) {
			std::cout<< "The number of rows of the destination matrix variable and the model matrix C is different!\n" << std::endl;
		}
	if else (C.cols() != Css.cols()) {
			std::cout<< "The number of columns of the destination matrix variable and the model matrix C is different!\n" << std::endl;
		}
	else {
			C = /*mpc::test_models::Tanksystem*/ Css.transpose(); 
		}
} // end of routine getModelParameterC
