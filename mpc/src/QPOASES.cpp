#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>
#include <mpc/optimizer/QPOASES.h>
#include <mpc/model/model.h>
#include <mpc/optimizer/optimizer.h>

using namespace Eigen;

mpc::optimizer::QPOASES::QPOASES(mpc::model::Model *model_ptr)
{
	model_ = model_ptr;
}

void mpc::optimizer::QPOASES::setOptimizationParams(int n, int np, int p, double H_[], double F_[])//mpc::model::Model *model_ptr
{

// Obtention of the model parameters

MatrixXd Ass(n,n);
model_->getModelParameterA(Ass);

MatrixXd Bss(n,p);
model_->getModelParameterB(Bss);

MatrixXd Qss(n,n);
Qss = MatrixXd::Identity(n,n);
MatrixXd Rss(n,n);
Rss = MatrixXd::Identity(n,n);

// TODO methods to obtain the matrices Q, P and R to create the extended matrices

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
	int z=0;
	// Assignment of each individual element of the base vector
	ROS_INFO("Base size: %i", (int) base.size());
	//TODO: I dont remember whe I can initialize int k!!!
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

MatrixXd R(np*p, np*p);

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

// Creation of the H and F matrices and assignment to a standard C array
MatrixXd H(np*p, np*p);
H = B.transpose()*Q*B + R;

MatrixXd F(np*p, np*p);
F = B.transpose()*Q*A;


double * H_ptr;
double * F_ptr;

H_ptr = H.data();
F_ptr = F.data();

for (int t=0; t < (H.rows()*H.cols()); t++){	
		H_[t] = *H_ptr;
		F_[t] = *F_ptr;
		H_ptr++;
		F_ptr++;
}


} // End of routine setOptimizationParams

/*void QPOASES::computeMPC(mpc::model::Model *model, int &nWSR, double *cputime)
{



}*/
