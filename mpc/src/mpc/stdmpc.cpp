#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <mpc/mpc/stdmpc.h>
#include <mpc/model/model.h>




mpc::STDMPC::STDMPC(ros::NodeHandle node_handle) : nh_(node_handle)//, model_(0), optimizer_(0), simulator_(0)
{
	model_ = 0;
	optimizer_ = 0;
	simulator_ = 0;
}


mpc::STDMPC::~STDMPC()
{

}


bool mpc::STDMPC::resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator)
{
	// setting of the pointer of the model, optimizer and simulator classes
	model_ = model;
	optimizer_ = optimizer;
	simulator_ = simulator;
	
	// reading of the horizon value of the model predictive control algorithm
	nh_.param<int>("horizon", horizon_, 30);
	ROS_INFO("Got param: horizon = %d", horizon_);

	// reading of the problem variables
	states_ = model_->getStatesNumber();
	inputs_ = model_->getInputsNumber();
	outputs_ = model_->getOutputsNumber();

	variables_ = horizon_ * inputs_;
	optimizer_->setHorizon(horizon_);
	optimizer_->setVariableNumber(variables_);

	if (!optimizer_->init()) {
		ROS_INFO("Could not initialized the optimizer class.");
		return false;
	}
	constraints_ = optimizer_->getConstraintNumber();


	ROS_INFO("Reset successful.");
	return true;
}



bool mpc::STDMPC::initMPC()
{
	ROS_INFO("states: %i inputs: %i constraints: %i horizon: %i variables: %i", states_, inputs_, constraints_, horizon_, variables_);
	
	// Initialization of MPC solution
	mpc_solution_ = new double[variables_];
		
	// Initialization of state space matrices
	A_ = Eigen::MatrixXd::Zero(states_, states_);
	B_ = Eigen::MatrixXd::Zero(states_, inputs_);
	C_ = Eigen::MatrixXd::Zero(outputs_, states_);
	
	// Obtention of the model parameters
	model_->computeDynamicModel(A_, B_, C_);
	
	
	// Reading the weight matrices of the cost function
	Q_ = Eigen::MatrixXd::Zero(states_, states_);
	P_ = Eigen::MatrixXd::Zero(states_, states_);
	R_ = Eigen::MatrixXd::Zero(inputs_, inputs_);
	XmlRpc::XmlRpcValue Q_list, P_list, R_list;
	nh_.getParam("optimizer/states_error_weight_matrix/data", Q_list);
	ROS_ASSERT(Q_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(Q_list.size() == states_ * states_);

	nh_.getParam("optimizer/terminal_state_weight_matrix/data", P_list);
	ROS_ASSERT(P_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(P_list.size() == states_ * states_);
	
	int z = 0;
	for (int i = 0; i < states_; i++) {
		for (int j = 0; j < states_; j++) {
			ROS_ASSERT(Q_list[z].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			Q_(i, j) = static_cast<double>(Q_list[z]);
			
			ROS_ASSERT(P_list[z].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			P_(i, j) = static_cast<double>(P_list[z]);
			z++;
		}
	}
	
	nh_.getParam("optimizer/input_error_weight_matrix/data", R_list);
	ROS_ASSERT(R_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(R_list.size() == inputs_ * inputs_);
	z = 0;
	for (int i = 0; i < inputs_; i++) {
		for (int j = 0; j < inputs_; j++) {
			ROS_ASSERT(R_list[z].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			R_(i, j) = static_cast<double>(R_list[z]);
			z++;
		}
	}
	
	
	// Creation of the states and inputs weight matrices for the quadratic program
	Q_bar_ = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, (horizon_ + 1) * states_);
	R_bar_ = Eigen::MatrixXd::Zero(horizon_ * inputs_, horizon_ * inputs_);
	for (int i = 0; i < horizon_; i++) {
		Q_bar_.block(i * states_, i * states_, states_, states_) = Q_;
		R_bar_.block(i * inputs_, i * inputs_, inputs_, inputs_) = R_;
	}
	Q_bar_.block(horizon_ * states_, horizon_ * states_, states_, states_) = P_;
	
	
	// Reading the constraint vectors
	Eigen::VectorXd lbG = Eigen::VectorXd::Zero(constraints_);
	Eigen::VectorXd ubG = Eigen::VectorXd::Zero(constraints_);
	XmlRpc::XmlRpcValue lbG_list, ubG_list;
	nh_.getParam("optimizer/constraints/constraint_vector_low", lbG_list);
	ROS_ASSERT(lbG_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	nh_.getParam("optimizer/constraints/constraint_vector_upp", ubG_list);
	ROS_ASSERT(ubG_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	if (ubG_list.size() == lbG_list.size()) {
		for (int i = 0; i < ubG_list.size(); ++i) {
			ROS_ASSERT(lbG_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			lbG(i) = static_cast<double>(lbG_list[i]);
			
			ROS_ASSERT(ubG_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			ubG(i) = static_cast<double>(ubG_list[i]);
		}

	}
	
	// Reading the bound vectors
	Eigen::VectorXd lb = Eigen::VectorXd::Zero(inputs_);
	Eigen::VectorXd ub = Eigen::VectorXd::Zero(inputs_);
	XmlRpc::XmlRpcValue lb_list, ub_list;
	nh_.getParam("optimizer/constraints/bound_vector_low", lb_list);
	ROS_ASSERT(lb_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	nh_.getParam("optimizer/constraints/bound_vector_upp", ub_list);
	ROS_ASSERT(ub_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	if (ub_list.size() == lb_list.size()) {
		for (int i = 0; i < ub_list.size(); ++i) {
			ROS_ASSERT(lb_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			lb(i) = static_cast<double>(lb_list[i]);                              
			
			ROS_ASSERT(ub_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			ub(i) = static_cast<double>(ub_list[i]);
		}
	}
	
	// Reading the state bound matrix
	double m[constraints_ * states_];
	XmlRpc::XmlRpcValue M_list;
	nh_.getParam("optimizer/constraints/constraint_matrix_M", M_list);  
	ROS_ASSERT(M_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	for (int i = 0; i < M_list.size(); ++i) {
		ROS_ASSERT(M_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		m[i] = static_cast<double>(M_list[i]);
	}
	Eigen::Map<Eigen::MatrixXd, Eigen::RowMajor> M(m, constraints_, states_);
	
	
	// Creation of the extended constraint and bound vector
	lbG_bar_ = Eigen::VectorXd::Zero(constraints_ * horizon_);
	ubG_bar_ = Eigen::VectorXd::Zero(constraints_ * horizon_);
	lb_bar_ = Eigen::VectorXd::Zero(inputs_ * horizon_);
	ub_bar_ = Eigen::VectorXd::Zero(inputs_ * horizon_);
	for (int i = 0; i < horizon_; i++) {
		lbG_bar_.block(i * constraints_, 0, constraints_, 1) = lbG;
		ubG_bar_.block(i * constraints_, 0, constraints_, 1) = ubG;
		lb_bar_.block(i * inputs_, 0, inputs_, 1) = lb;
		ub_bar_.block(i * inputs_, 0, inputs_, 1) = ub;
	}
	
	// Creation of the extended constraint matrix G_bar_
	M_bar_ = Eigen::MatrixXd::Zero(constraints_ * horizon_, (horizon_ + 1) * states_);
	for (int i = 0; i < horizon_; i++) {
		for (int j = 0; j < horizon_ + 1; j++) {
			if (i == j) {
				M_bar_.block(i * constraints_, j * states_, constraints_, states_) = M;
			}
		}
	}
	
	
	ROS_INFO("STDMPC class successfully initialized.");
	return true;
}



void mpc::STDMPC::updateMPC(double* x_measured, double* x_reference)
{
	Eigen::Map<Eigen::VectorXd> x_measured_eigen(x_measured, states_, 1);
	Eigen::Map<Eigen::VectorXd> x_reference_eigen(x_reference, states_, 1);
	
	// Compute steady state control based on updated system matrices
	Eigen::JacobiSVD<Eigen::MatrixXd> SVD_B(B_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXd u_reference = SVD_B.solve(x_reference_eigen - A_ * x_reference_eigen);
	
	// Creation of the base vector
	A_pow_.push_back(Eigen::MatrixXd::Identity(states_, states_));
	for (int i = 1; i < horizon_ + 1; i++) {
		Eigen::MatrixXd A_pow_i = A_pow_[i-1] * A_;
		A_pow_.push_back(A_pow_i);
	}

	// Compute the hessian matrix and gradient vector for the quadratic program	
	A_bar_ = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, states_);
	B_bar_ = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, horizon_ * inputs_);
//	Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, horizon_ * states_);
	Eigen::MatrixXd x_ref_bar = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, 1);
	for (int i = 0; i < horizon_ + 1; i++) {
		for (int j = 0; j < horizon_; j++) {
			if (i == horizon_) {
				if (j == 0)
					A_bar_.block(i * states_, 0, states_, states_) = A_pow_[i];
				if (j == 0)
					x_ref_bar.block(i * states_, 0, states_, 1) = x_reference_eigen;
				if (i > j) {
					B_bar_.block(i * states_, j * inputs_, states_, inputs_) = A_pow_[i-j-1] * B_;
//					H_x.block(i * states_, j * states_, states_, states_) = A_p_BK_pow_[i-j-1];
				}
			}
			else {
				if (j == 0) {
					A_bar_.block(i * states_, 0, states_, states_) = A_pow_[i];
					x_ref_bar.block(i * states_, 0, states_, 1) = x_reference_eigen;
				}
				if (i > j) {
					B_bar_.block(i * states_, j * inputs_, states_, inputs_) = A_pow_[i-j-1] * B_;
//					H_x.block(i * states_, j * states_, states_, states_) = A_p_BK_pow_[i-j-1];
				}
			}
		}
	}
	double hessian_matrix[horizon_ * inputs_][horizon_ * inputs_];
	double gradient_vector[horizon_ * inputs_];
	Eigen::Map<Eigen::MatrixXd, Eigen::RowMajor> H(&hessian_matrix[0][0], horizon_ * inputs_, horizon_ * inputs_);
	Eigen::Map<Eigen::MatrixXd> g(gradient_vector, horizon_ * inputs_, 1);
	
	// Computing the values of the Hessian matrix and Gradient vector
	H = B_bar_.transpose() * Q_bar_ * B_bar_ + R_bar_;
	g = B_bar_.transpose() * Q_bar_ * A_bar_ * x_measured_eigen - B_bar_.transpose() * Q_bar_ * x_ref_bar;
	
	
	// Transforming constraints and bounds to array
	double lbG_bar[constraints_ * horizon_];
	double ubG_bar[constraints_ * horizon_];
	double lb_bar[horizon_ * inputs_];
	double ub_bar[horizon_ * inputs_];
	Eigen::Map<Eigen::VectorXd> lbG_bar_eigen(lbG_bar, constraints_ * horizon_);
	Eigen::Map<Eigen::VectorXd> ubG_bar_eigen(ubG_bar, constraints_ * horizon_);
	Eigen::Map<Eigen::VectorXd> lb_bar_eigen(lb_bar, inputs_ * horizon_);
	Eigen::Map<Eigen::VectorXd> ub_bar_eigen(ub_bar, inputs_ * horizon_);
	lbG_bar_eigen = lbG_bar_ - M_bar_ * A_bar_ * x_measured_eigen;
	ubG_bar_eigen = ubG_bar_ - M_bar_ * A_bar_ * x_measured_eigen;
	lb_bar_eigen = lb_bar_;
	ub_bar_eigen = ub_bar_;

	
	// Mapping of the extended constraint matrix G_bar_
	double constraint_matrix[horizon_ * constraints_][horizon_ * inputs_];
	Eigen::Map<Eigen::MatrixXd, Eigen::RowMajor> G_bar(&constraint_matrix[0][0], constraints_ * horizon_, horizon_ * inputs_);
	G_bar = M_bar_ * B_bar_;
	
	
	double cputime = 0;//1.0;//NULL;
	bool success = false;
	success = optimizer_->computeOpt(&hessian_matrix[0][0], gradient_vector, &constraint_matrix[0][0], lb_bar, ub_bar, lbG_bar, ubG_bar, cputime);
	if (success){
		mpc_solution_ = optimizer_->getOptimalSolution();
		for (int i = 0; i < variables_; i++){
			std::cout <<"Solution vector: solution["<< i <<"] = " << *(mpc_solution_ + i) << std::endl;
		}
	}
	else {	
		ROS_WARN("An optimal solution could not be obtained.");
	}

}


