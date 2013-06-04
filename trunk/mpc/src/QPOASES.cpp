 	#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <mpc/optimizer/QPOASES.h>
#include <mpc/model/model.h>
#include <mpc/optimizer/optimizer.h>


mpc::optimizer::QPOASES::QPOASES(mpc::model::Model *model_ptr, int n, int p, int horizon)
{
	model_ = model_ptr;
	n_ = n;
	p_ = p;
	horizon_ = horizon;
}




// TODO Define a function to read from a .yaml file to get nWSR and cputime variables

void mpc::optimizer::QPOASES::computeMPC(Eigen::VectorXd x_k, Eigen::VectorXd x_ref)
{
	Eigen::MatrixXd Ass(n_,n_);
	Eigen::MatrixXd Bss(n_,p_);

	// Obtention of the model parameters
	model_->getModelParameterA(Ass);
	model_->getModelParameterB(Bss);

	Eigen::MatrixXd Qss(n_,n_);
	Eigen::MatrixXd Pss(n_,n_);
	Eigen::MatrixXd Rss(p_,p_);
	//Qss = Eigen::MatrixXd::Identity(n_,n_);
	//Rss = Eigen::MatrixXd::Identity(p_,p_);


	
	ros::NodeHandle n;

	//Fetch parameters for matrix Q
	if (n.getParam("/mpc/optimizer/Qss_one_one", Qss(0,0)))
	{
		ROS_INFO("Got param: %f", Qss(0,0));
	}
	
	if (n.getParam("mpc/optimizer/Qss_one_two", Qss(0,1)))
	{
		ROS_INFO("Got param: %f", Qss(1,0)); //Qss(1,0) = Qss_two_one;
	}

	if (n.getParam("mpc/optimizer/Qss_two_one", Qss(1,0)))
	{
		ROS_INFO("Got param: %f", Qss(1,0)); //Qss(1,1) = Qss_two_two;
	}
	
	if (n.getParam("mpc/optimizer/Qss_two_two", Qss(1,1)))
	{
		ROS_INFO("Got param: %f", Qss(1,1)); //Qss(1,1) = Qss_two_two;
	}

	// Fetch parameters for matrix R
	if (n.getParam("mpc/optimizer/Rss_one_one", Rss(0,0)))
	{
  		ROS_INFO("Got param: %f", Rss(0,0)); //Rss(0,0) = Rss_one_one;
	}

	// Fetch parameters for matrix P
	if (n.getParam("mpc/optimizer/Pss_one_one", Pss(0,0)))
	{
  		ROS_INFO("Got param: %f", Pss(0,0)); //Pss(0,0) = Pss_one_one;
	}

	if (n.getParam("mpc/optimizer/Pss_one_two", Pss(0,1)))
	{
  		ROS_INFO("Got param: %f", Pss(0,1)); //Pss(0,0) = Pss_one_one;
	}

	if (n.getParam("mpc/optimizer/Pss_two_one", Pss(1,0)))
	{
  		ROS_INFO("Got param: %f", Pss(1,0)); //Pss(0,0) = Pss_one_one;
	}

	if (n.getParam("mpc/optimizer/Pss_tow_two", Pss(1,1)))
	{
  		ROS_INFO("Got param: %f", Pss(1,1)); //Pss(0,0) = Pss_one_one;
	}

std::cout << Qss <<" = Qss" << std::endl;
std::cout << Pss <<" = Pss" << std::endl;
std::cout << Rss <<" = Rss" << std::endl;



	//CREATION OF THE A MATRIX
	Eigen::MatrixXd A((horizon_ + 1) * n_, n_);
	Eigen::MatrixXd An(n_, n_);

    //Assign the identity matrix to the first position of the matrix A
    A.block(0,0,n_,n_) = Eigen::MatrixXd::Identity(n_,n_); 

    //Loop to fill the A matrix with the corresponding power of Ass
	Eigen::MatrixXd temp_1(n_,n_);
	temp_1 = Eigen::MatrixXd::Identity(n_,n_);
    for (int i = 1; i <= horizon_; i++) {
		// Loop to raise the current matrix to the j-th power
		An = Ass;	
		temp_1 = An * temp_1;
		An = temp_1;
	
		A.block(i*n_,0,n_,n_) = An;
    }

std::cout << A <<" = A_bar" << std::endl;


	//CREATION OF THE B MATRIX
	Eigen::MatrixXd B((horizon_ + 1) * n_, horizon_ * p_);
	Eigen::MatrixXd Bn(n_,n_);
	Eigen::MatrixXd temp_2(n_,n_);
	temp_2 = Eigen::MatrixXd::Identity(n_,n_);

	//Creation of the standard vector to define the first row
	std::vector<Eigen::MatrixXd> base;
	base.push_back(Eigen::MatrixXd::Zero(n_,p_));
	base.push_back(Bss);


		//CREATING THE BASE VECTOR
	Eigen::MatrixXd aux(n_,p_);
	//Loop to fill the rest of the elements of the vector
	for (int ii = 0; ii < horizon_ - 1; ii++) {
		//Loop to perform the power raising of the A matrix in each case
								
		// Recursive multiplication to raise to the i-th power		
		Bn = Ass;
		temp_2 = Bn*temp_2;
		Bn = temp_2;
		
		aux = Bn * Bss; 
		base.push_back(aux);
	}




	//USING THE BASE VECTOR TO FILL UP THE B MATRIX
	// Loop to assign and resize the base vector to matrix B
	for (int j = 0; j < horizon_; j++) {
		int z = 0;
		// Assignment of each individual element of the base vector
		for(int k = j; k < (int) base.size(); k++) {
			B.block(k*n_, j*p_, n_, p_) = base[z];
			z++;
		}
	}
	std::cout << B << " = B_bar" << std::endl;
	


	//CREATION OF THE Q MATRIX
	Eigen::MatrixXd Q((horizon_ + 1) * n_, (horizon_ + 1) * n_);

	for (int q = 0; q < horizon_ + 1; q++) {
		for (int qq = 0; qq < horizon_ + 1; qq++) {
			if (q == qq)
				Q.block(q*n_,qq*n_,n_,n_) = Qss;
			else
				Q.block(q*n_,qq*n_,n_,n_) = Eigen::MatrixXd::Zero(n_,n_);
		}
	} // TODO add the P matrix to the last position of the Q matrix

	std::cout << Q << " = Q_bar" << std::endl;

	//CREATION OF THE R MATRIX
	Eigen::MatrixXd R(horizon_ * p_, horizon_ * p_);

	for (int r = 0; r < horizon_; r++) {
		for (int rr = 0; rr < horizon_; rr++) {
			if (r == rr)
				R.block(r*p_,rr*p_,p_,p_) = Rss;
			else
				R.block(r*p_,rr*p_,p_,p_) = Eigen::MatrixXd::Zero(p_,p_);
		}
	}

	std::cout << R << " = R_bar" << std::endl;

	// Creation of the H and g matrices and assignment to a standard C array
	Eigen::MatrixXd H(horizon_ * p_, horizon_ * p_);
	H = B.transpose()*Q*B + R;
	std::cout << H << " = H" << std::endl;

	Eigen::MatrixXd g(horizon_ * p_, 1);

	Eigen::MatrixXd x_ref_bar(n_ * (horizon_ + 1), 1);	
	for (int i=0; i<(horizon_ + 1); i++){
		x_ref_bar.block(n_*i, 0, n_, 1) = x_ref;
	}
	std::cout << x_ref_bar << " = x_ref_bar" << std::endl;

	g = B.transpose()*Q*A*x_k - B.transpose()*Q*x_ref_bar;
	std::cout << g << " = g" << std::endl;


	double * H_ptr;
	double * g_ptr;

	H_ptr = H.data();
	g_ptr = g.data();

	double H_array[horizon_*p_*horizon_*p_];
	double g_array[horizon_*p_];	

	for (int t = 0; t < (H.rows() * H.cols()); t++) {
		H_array[t] = *H_ptr;
		H_ptr++;	
	}

	for (int v = 0; v < (g.rows() * g.cols()); v++){
		g_array[v] = *g_ptr;
		g_ptr++;
	}

	for (int i = 0; i < horizon_ * p_ * horizon_ * p_; i++) {
		std::cout<< "H[" << i << "] = "<< H_array[i] << std::endl;
	}

	for (int j = 0; j < horizon_ * p_; j++) {
		std::cout<< "g[" << j << "] = "<< g_array[j] << std::endl;
	}

}
