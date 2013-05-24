#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <mpc/optimizer/QPOASES.h>
#include <mpc/model/model.h>
#include <mpc/optimizer/optimizer.h>


mpc::optimizer::QPOASES::QPOASES(mpc::model::Model *model_ptr)
{
	model_ = model_ptr;
}

void mpc::optimizer::QPOASES::setOptimizationParams(int n, int horizon, int p, double H_matrix[], double F_matrix[])
{
	Eigen::MatrixXd Ass(n,n);
	Eigen::MatrixXd Bss(n,p);

	// Obtention of the model parameters
	model_->getModelParameterA(Ass);
	model_->getModelParameterB(Bss);

	Eigen::MatrixXd Qss(n,n);
	Eigen::MatrixXd Rss(p,p);
	Qss = Eigen::MatrixXd::Identity(n,n);
	Rss = Eigen::MatrixXd::Identity(p,p);

	// TODO methods to obtain the matrices Q, P and R to create the extended matrices



	//CREATION OF THE A MATRIX
	Eigen::MatrixXd A((horizon + 1) * n, n);
	Eigen::MatrixXd An(n, n);

    //Assign the identity matrix to the first position of the matrix A
    A.block(0,0,n,n) = Eigen::MatrixXd::Identity(n,n); 

    //Loop to fill the A matrix with the corresponding power of Ass
	Eigen::MatrixXd temp_1(n,n);
	temp_1 = Eigen::MatrixXd::Identity(n,n);
    for (int i = 1; i <= horizon; i++) {
		// Loop to raise the current matrix to the j-th power
		An = Ass;	//TODO: function that return the A matrix from the state space model in the required time instant
		temp_1 = An * temp_1;
		An = temp_1;
	
		A.block(i*n,0,n,n) = An;
    }

std::cout << A <<" = A_bar" << std::endl;


	//CREATION OF THE B MATRIX
	Eigen::MatrixXd B((horizon + 1) * n, horizon * p);
	Eigen::MatrixXd Bn(n,n);
	Eigen::MatrixXd temp_2(n,n);
	temp_2 = Eigen::MatrixXd::Identity(n,n);

	//Creation of the standard vector to define the first row
	std::vector<Eigen::MatrixXd> base;
	base.push_back(Eigen::MatrixXd::Zero(n,p));
	base.push_back(Bss); //TODO: function that return the B matrix from the state space model in the required time instant


		//CREATING THE BASE VECTOR
	Eigen::MatrixXd aux(n,p);
	//Loop to fill the rest of the elements of the vector
	for (int ii = 0; ii < horizon - 1; ii++) {
		//Loop to perform the power raising of the A matrix in each case
								
		// Recursive multiplication to raise to the i-th power		
		Bn = Ass;
		temp_2 = Bn*temp_2;
		Bn = temp_2;
		
		aux = Bn * Bss; //TODO: function that return the A matrix from the state space model in the required time instant
		base.push_back(aux);
	}
// FIXME DEBUGGING CODE std::cout<< "Size of the base vector:\n" << base.size() << std::endl;



	//USING THE BASE VECTOR TO FILL UP THE B MATRIX
	// Loop to assign and resize the base vector to matrix B
	for (int j = 0; j < horizon; j++) {
		int z = 0;
		// Assignment of each individual element of the base vector
		for(int k = j; k < (int) base.size(); k++) {
			B.block(k*n, j*p, n, p) = base[z];
			z++;
		}
	}
	std::cout << B << " = B_bar" << std::endl;
	


	//CREATION OF THE Q MATRIX
	Eigen::MatrixXd Q((horizon + 1) * n, (horizon + 1) * n);

	for (int q = 0; q < horizon + 1; q++) {
		for (int qq = 0; qq < horizon + 1; qq++) {
			if (q == qq)
				Q.block(q*n,qq*n,n,n) = Qss;
			else
				Q.block(q*n,qq*n,n,n) = Eigen::MatrixXd::Zero(n,n);
		}
	}

	std::cout << Q << " = Q_bar" << std::endl;

	//CREATION OF THE R MATRIX
	Eigen::MatrixXd R(horizon * p, horizon * p);

	for (int r = 0; r < horizon; r++) {
		for (int rr = 0; rr < horizon; rr++) {
			if (r == rr)
				R.block(r*p,rr*p,p,p) = Rss;
			else
				R.block(r*p,rr*p,p,p) = Eigen::MatrixXd::Zero(p,p);
		}
	}

	std::cout << R << " = R_bar" << std::endl;

	// Creation of the H and F matrices and assignment to a standard C array
	Eigen::MatrixXd H(horizon * p, horizon * p);
	H = B.transpose()*Q*B + R;
	std::cout << H << " = H" << std::endl;

	Eigen::MatrixXd F(horizon * p, n);
	F = B.transpose()*Q*A;
	std::cout << F << " = F" << std::endl;


	double * H_ptr;
	double * F_ptr;

	H_ptr = H.data();
	F_ptr = F.data();

	for (int t = 0; t < (H.rows() * H.cols()); t++) {
		H_matrix[t] = *H_ptr;
		H_ptr++;	
	}

	for (int v = 0; v < (F.rows() * F.cols()); v++){
		F_matrix[v] = *F_ptr;
		F_ptr++;
	}

} // End of routine setOptimizationParams


/*void QPOASES::computeMPC(mpc::model::Model *model, int &nWSR, double *cputime)
{

}*/
