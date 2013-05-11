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





}
