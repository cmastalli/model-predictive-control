#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>
#include "Solver_qpOASES"

void setOptimizationParams(mpc::model::Model *model, MatrixXd &H_, MatrixXd &F_)
{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CREATION OF THE A MATRIX

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MatrixXd A((np+1)*n, n);
MatrixXd An(n,n);

    //Assign the identity matrix to the first position of the matrix A
    A.block(0,0,n,n) =  MatrixXd::Identity(n,n); 

    //Loop to fill the A matrix with the corresponding power of Ass
	MatrixXd temp(n,n);
	temp = MatrixXd::Identity(n, n);    
    for (int i = 1; i <= np; i++)
    {
		// Loop to raise the current matrix to the j-th power
		An = Ass;//TODO: function that return the A matrix from the state space model in the required time instant
		temp = An*temp;
		An = temp;
	
		A.block(i*n,0,n,n) = An;
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CREATION OF THE B MATRIX

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MatrixXd B((np+1)*n, np*p);

//Creation of the standard vector to define the first row
std::vector<MatrixXd> base;

//Fill the initial element of the vector with the corresponding zero matrix
base.push_back(MatrixXd::Zero(n,p));

//Fill the second element of the vector with the Bss matrix
base.push_back(Bss);

//Auxiliar initializations for the power raising operation
MatrixXd Bn(n,n);

MatrixXd tempII(n,n);
tempII = MatrixXd::Identity(n,n);

/*
CREATING THE BASE VECTOR
*/

MatrixXd aux(n,p);

	//Loop to fill the rest of the elements of the vector
for(int ii=0; ii < (np-1); ii++){
		 
	//Loop to perform the power raising of the A matrix in each case
								
	// Recursive multiplication to raise to the i-th power		
	Bn = Ass;
	tempII = Bn*tempII;
	Bn = temp;	
			
	
	std::cout<< "Matrix An:\n" << Bn << std::endl;
	aux = Bn*Bss; 
	base.push_back(aux);
		
}
// FIXME DEBUGGING CODE std::cout<< "Size of the base vector:\n" << base.size() << std::endl;


/*
USING THE BASE VECTOR TO FILL UP THE B MATRIX
*/

// Loop to assign and resize the base vector to matrix B
for(int j=0; j < np; j++){
	// Assignment of each individual element of the base vector
	int z = 0;
	ROS_INFO("Base size: %i", (int) base.size());
	for(int k=j; k < (int)base.size(); k++){
			
		B.block(k*n, j*p, n, p) = base[z];
			
		//FIXME DEBUGGING CODE std::cout<< "Here is the"<< z <<"th element of vector B:\n"<< base[z] << std::endl;			
		z++;
	}
		
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CREATION OF THE Q MATRIX

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MatrixXd Q((np+1)*n, (np+1)*n);

for (int q=0; q<np+1; q++){
		
	for (int qq=0; qq<np+1; qq++){
			
		//std::cout<< "Current value of i:\n" << i << std::endl;
		//std::cout<< "Current value of j:\n" << j << std::endl;
		if (q==qq){
			Q.block(q*n,qq*n,n,n) = Qss;
			
		}
		else{
			Q.block(q*n,qq*n,n,n) = MatrixXd::Zero(n,n);
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CREATION OF THE R MATRIX

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MatrixXd R(np*p, np*p)

for (int r=0; r<np; r++){
		
	for (int rr=0; rr<np; rr++){
			
		//std::cout<< "Current value of i:\n" << i << std::endl;
		//std::cout<< "Current value of j:\n" << j << std::endl;
		if (r==rr){
			R.block(r*p,rr*p,p,p) = Rss;
		}
		else{
			R.block(r*p,rr*p,p,p) = MatrixXd::Zero(p,p);
		}
	}
}

 //TODO create the routine that computes matrix H and F in the optimization problem

H_ = B.transpose()*Q*B + R;

F_ = B.transpose()*Q*A;


} // End of routine setOptimizationParams

void computeMPC(mpc::model::Model *model, int &nWSR, double *cputime)
{



}
