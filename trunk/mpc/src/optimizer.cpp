#include <iostream>
//#include "model.h"
#include <Eigen/Core>
#include "optimizer.h"
//#include "simulator.h"
//#include "model_predictive_control.h"

using namespace mpc;

void setOptimizationParams(MPC::Model *model);
{
/* Global variables to be used in the manipulation 

	int n	size of the state vector of the state space model
	int p	size of the input vector of the state space model
	int q	size of the output vector of the state space model
	int np	number of samples of the prediction horizon

TODO Add a tag to identify these variables as global variables
*/

// Ass ---> The "A" matrix of the State Space representation
MatrixXd Ass(n,n) = model->A; //TODO specify a way to define the parameters to be fed to this function in the "model" class
MatrixXd Bss(n,p) = model->B;
MatrixXd Css(q,p) = model->C;

createA();
createB();

//TODO specify the functions to create the remaining matrices required to set the optimization problem




}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// To create Matrix A
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void createA(int n, int np, MatrixXd Ass);{
    MatrixXd A(n*(np + 1), n);
	MatrixXd An(n,n);

    //Assign the identity matrix to the first position of the matrix A
    A.block<n,n>(0,0) =  MatrixXd::Identity(n,n); //TODO define the Identity matrix function

    //Loop to fill the A matrix with the corresponding power of Ass
	MatrixXd temp(n,n) =  MatrixXd::Identity(n, n);    
    for (int i = 1; i <= np; i++)
    {
		// Loop to raise the current matrix to the j-th power
		for (int j = 1; j <= np; j++) {
			An = Ass;//TODO: function that return the A matrix from the state space model in the required time instant
			An = temp*An;
		}

    A.block<n,n>(i*n,0) = An;
    }
}			

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// To create Matrix B
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*void createB(int n, int np, int p, MatrixXd Bss, MatrixXd Ass);{

    //B will be created from the coefficient wise multiplication of two matrices B1 and B2
    MatrixXd B1((np+1)*n,np*p);

    // Loop to fill the B1 matrix as a triangular lower matrix where all the coefficients are equal to Bss
    for (int i=0, i<=(np+1)*n; i++)
    {
	for (int j=0; j<=np*p; j++)
	{
	if (i>=j){
		B1.block<n,p>(i,j)= Bss;
		}
	else {
		B1.block<n,p>(i,j)= Zero(n,p); //TODO define the zero matrix function
		}
	}

    //Matrix B2 containing the corresponding powers of A
    MatrixXd B2((np+1)*n,np*p);

    //Loop to assign the corresponding powers of A to the B2 matrix
    for (int i2= 0; i2<=(np+1)*n; i2++)
    {
	for (int j2=0; j2<=np*p; j2++)
	{
	if (i2<j2){
		B2.block<>
*/		




