#include <ros/ros.h>

#include <Eigen/Dense>

#include <mpc/optimizer/lbmpc_qpoases.h>


mpc::LBMPC::LMPC()
{

}


mpc::LBMPC::initMPC()
{
//	model_ = model_ptr;

	states_ = states;
	inputs_ = inputs;
	horizon_ = horizon;
	
	A_nominal_ = MatrixXd::Zero(states_, states_);
	A_estimated_ = MatrixXd::Zero(states_, states_);
	
	B_nominal_ = MatrixXd::Zero(states_, inputs_);
	B_estimated_ = MatrixXd::Zero(states_, inputs_);
	
	d_nominal_ = MatrixXd::Zero(inputs_, 1);
	d_estimated_ = MatrixXd::Zero(inputs_, 1);
	
	Q_ = MatrixXd::Zero(states_, states_);
	P_ = MatrixXd::Zero(states_, states_);
	R_ = MatrixXd::Zero(inputs_, inputs_);

	//TODO: To get numerical values of Q, R and P matrices through the parameter serves
}


mpc::LBMPC::computeMPC(double* x_measured, double* x_reference)
{
	Map<Matrix<double, states_, 1> > x_measured_eigen(x_measured, states_, 1);
	Map<Matrix<double, states_, 1> > x_reference_eigen(x_reference, states_, 1);

	if (enable_learning_process_) {
		//TODO a function that determines the matrix of model learned, i.e. A_learned and B_learned
		
		A_estimated_ = A_nominal_ + A_learned;
		B_estimated_ = B_nominal_ + B_learned;
		d_estimated_ = d_nominal_ + d_learned;
	}


	Eigen::MatrixXd A_p_Bk = A_estimated_ + B_estimated_ * K_;
	A_p_BK_pow_.push_back(Eigen::MatrixXd::Identity(states_, states_));
	for (int i = 1; i < horizon_ + 1; i++) {
		Eigen::MatrixXd A_p_BK_pow_i = A_p_BK_pow_[i-1] * A_p_Bk;
		
		A_p_BK_pow_.push_back(A_p_BK_pw_i);
	}
	
	Eigen::MatrixXd A_x((horizon_ + 1) * states_, states_);
	Eigen::MatrixXd B_x = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, horizon_ * inputs_);
	Eigen::MatrixXd A_u((horizon_ + 1) * inputs_, states_);
	Eigen::MatrixXd B_u = Eigen::MatrixXd::Zero((horizon_ + 1) * inputs_, horizon_ * inputs_);
	for (int i = 0; i < horizon_ + 1; i++) {
		for (int j = 0; j < horizon_; j++) {
			if (j == 0) {
				A_x.block(i * states_, 0, states_, states_) = A_p_BK_pow_[i];
				A_u.block(i * states_, 0, states_, states_) = K_ * A_p_BK_pow_[i];
			}
			if (i == j)
				B_u.block(i * states_, j * inputs_, states_, inputs_) = Eigen::MatrixXd::Identity(states_, inputs_);
				
			if (i > j) {
				B_x.block(i * states_, j * inputs_, states_, inputs_) = A_p_BK_pow_[i-j-1] * B_estimated_;
				B_u.block(i * states_, j * inputs_, states_, inputs_) = K_ * A_p_BK_pow_[i-j-1] * B_estimated_;
			}
	
/*		int z = 0;
		for (int j = i; j < (int) A_p_BK_pow_.size(); j++) {
			B_x.block(j * states_, i * inputs_, states_, inputs_) = A_p_BK_pow_[z] * B_estimated_;
			//d_x.block() = ;
			
			z++;
		}*/
	}

}
