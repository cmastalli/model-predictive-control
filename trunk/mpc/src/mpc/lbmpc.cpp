#include <mpc/mpc/lbmpc.h>



mpc::LBMPC::LBMPC(ros::NodeHandle node) : nh_(node)
{
	model_ = 0;
	optimizer_ = 0;
	simulator_ = 0;
	enable_record_ = true;
}


bool mpc::LBMPC::resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator)
{
	model_ = model;
	optimizer_ = optimizer;
	simulator_ = simulator;
	
	// Reading of the horizon value of the model predictive control algorithm
	nh_.param<int>("horizon", horizon_, 30);
	ROS_INFO("Got param: horizon = %d", horizon_);
	
		// Ask about the enable or disable of the learning process
	if (!nh_.getParam("enable_learning_process", enable_learning_process_)) {
		enable_learning_process_ = false;
		ROS_WARN("Could not get the enable or disable learning process, therefore it is disable the learning process.");
	}
	
	
	nh_.param<int>("infeasibility_hack_counter_max", infeasibility_hack_counter_max_, 1);
	ROS_INFO("Got param: infeasibility_hack_counter_max = %d", infeasibility_hack_counter_max_);
	
	// Reading the path and data name
	if (!nh_.getParam("path_name", path_name_)) {
		ROS_WARN("The data will not save because could not found path name from parameter server.");
		enable_record_ = false;
	}
	if (!nh_.getParam("data_name", data_name_)) {
		ROS_WARN("The data will not save because could not found data name from parameter server.");
		enable_record_ = false;
	}
	
	// Get the number of states, inputs and outputs of the plant
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


bool mpc::LBMPC::initMPC()
{
	// Initialization of MPC solution
	mpc_solution_ = new double[variables_];
	control_signal_ = new double[inputs_];
	u_reference_ = Eigen::MatrixXd::Zero(inputs_, 1);
	infeasibility_counter_ = 0;
	x_.resize(states_);
	xref_.resize(states_);
	u_.resize(inputs_);
	
		
	// Get the nominal dynamic model matrices
	A_nominal_ = Eigen::MatrixXd::Zero(states_, states_);
	A_estimated_ = Eigen::MatrixXd::Zero(states_, states_);
	
	B_nominal_ = Eigen::MatrixXd::Zero(states_, inputs_);
	B_estimated_ = Eigen::MatrixXd::Zero(states_, inputs_);
	
	d_nominal_ = Eigen::MatrixXd::Zero(states_, 1);
	d_estimated_ = Eigen::MatrixXd::Zero(states_, 1);
	
	C_nominal_ = Eigen::MatrixXd::Zero(outputs_, states_);
	if (!model_->computeDynamicModel(A_nominal_, B_nominal_, C_nominal_)) {
		ROS_ERROR("Could not compute the nominal dynamic model of the linear system.");
		return false;
	}
	
	
	// Get the feedback gain that serves to limit the effects of model uncertainty
	K_ =  Eigen::MatrixXd::Zero(inputs_, states_);
	XmlRpc::XmlRpcValue feedback_gain_list;
	nh_.getParam("feedback_gain/data", feedback_gain_list);
	ROS_ASSERT(feedback_gain_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(feedback_gain_list.size() == inputs_ * states_);
	int z = 0;
	for (int i = 0; i < inputs_; i++) {
		for (int j = 0; j < states_; j++) {
			ROS_ASSERT(feedback_gain_list[z].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			K_(i, j) = static_cast<double>(feedback_gain_list[z]);
			z++;
		}
	}
	

	// Get the weight matrices of the cost function
	Q_ = Eigen::MatrixXd::Zero(states_, states_);
	P_ = Eigen::MatrixXd::Zero(states_, states_);
	R_ = Eigen::MatrixXd::Zero(inputs_, inputs_);

	//TODO: To get numerical values of Q, R and P matrices through the parameter serves
	XmlRpc::XmlRpcValue Q_list, P_list, R_list;
	nh_.getParam("optimizer/states_error_weight_matrix/data", Q_list);
	ROS_ASSERT(Q_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(Q_list.size() == states_ * states_);

	nh_.getParam("optimizer/terminal_state_weight_matrix/data", P_list);
	ROS_ASSERT(P_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(P_list.size() == states_ * states_);
	
	z = 0;
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
	
	
	ROS_INFO("Learning-based MPC successfully initialized");
	return true;
}


void mpc::LBMPC::updateMPC(double* x_measured, double* x_reference)
{
	
	Eigen::Map<Eigen::VectorXd> x_measured_eigen(x_measured, states_, 1);
	Eigen::Map<Eigen::VectorXd> x_reference_eigen(x_reference, states_, 1);

	// Compute the estimated dynamic model matrices
	if (enable_learning_process_) {
		//TODO a function that determines the matrix of model learned, i.e. A_learned and B_learned
		Eigen::MatrixXd A_learned = Eigen::MatrixXd::Zero(states_, states_);
		Eigen::MatrixXd B_learned = Eigen::MatrixXd::Zero(states_, inputs_);
		Eigen::MatrixXd d_learned = Eigen::MatrixXd::Zero(states_, 1);
		
		A_estimated_ = A_nominal_ + A_learned;
		B_estimated_ = B_nominal_ + B_learned;
		d_estimated_ = d_nominal_ + d_learned;
	}
	else {
		A_estimated_ = A_nominal_;
		B_estimated_ = B_nominal_;
		d_estimated_ = d_nominal_;
	}

	// Compute steady state control based on updated system matrices
	Eigen::JacobiSVD<Eigen::MatrixXd> SVD_B(B_estimated_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXd u_reference = SVD_B.solve(x_reference_eigen - A_estimated_ * x_reference_eigen - d_estimated_);


	Eigen::MatrixXd A_p_Bk = A_estimated_ + B_estimated_ * K_;
	A_p_BK_pow_.push_back(Eigen::MatrixXd::Identity(states_, states_));
	for (int i = 1; i < horizon_ + 1; i++) {
		Eigen::MatrixXd A_p_BK_pow_i = A_p_BK_pow_[i-1] * A_p_Bk;
		A_p_BK_pow_.push_back(A_p_BK_pow_i);
//		std::cout << A_p_BK_pow_i << " = (A+BK)^" << i << std::endl;//Work it!
	}

	// Compute the hessian matrix and gradient vector for the quadratic program	
	Eigen::MatrixXd A_x((horizon_ + 1) * states_, states_);
	Eigen::MatrixXd B_x = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, horizon_ * inputs_);
	Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, horizon_ * states_);
	
	Eigen::MatrixXd A_u(horizon_ * inputs_, states_);
	Eigen::MatrixXd B_u = Eigen::MatrixXd::Zero(horizon_ * inputs_, horizon_ * inputs_);
	Eigen::MatrixXd H_u = Eigen::MatrixXd::Zero(horizon_ * inputs_, horizon_ * states_);
	Eigen::MatrixXd u_s = Eigen::MatrixXd::Zero(horizon_ * inputs_, 1);
	Eigen::MatrixXd x_s = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, 1);
	Eigen::MatrixXd d_bar = Eigen::MatrixXd::Zero(horizon_ * states_, 1);
	for (int i = 0; i < horizon_ + 1; i++) {
		for (int j = 0; j < horizon_; j++) {
			if (i == horizon_) {
				if (j == 0)
					A_x.block(i * states_, 0, states_, states_) = A_p_BK_pow_[i];
				
				if (j == 0)
					x_s.block(i * states_, 0, states_, 1) = x_reference_eigen;
				
				if (i > j) {
					B_x.block(i * states_, j * inputs_, states_, inputs_) = A_p_BK_pow_[i-j-1] * B_estimated_;
					H_x.block(i * states_, j * states_, states_, states_) = A_p_BK_pow_[i-j-1];
				}
			}
			else {
				if (j == 0) {
					A_x.block(i * states_, 0, states_, states_) = A_p_BK_pow_[i];
					A_u.block(i * inputs_, 0, inputs_, states_) = K_ * A_p_BK_pow_[i];
					u_s.block(i * inputs_, 0, inputs_, 1) = u_reference;
					x_s.block(i * states_, 0, states_, 1) = x_reference_eigen;
				}
				if (i == j) {
					B_u.block(i * inputs_, j * inputs_, inputs_, inputs_) = Eigen::MatrixXd::Identity(inputs_, inputs_);
					d_bar.block(i * states_, 0, states_, 1) = d_estimated_;
				}
				
				if (i > j) {
					B_x.block(i * states_, j * inputs_, states_, inputs_) = A_p_BK_pow_[i-j-1] * B_estimated_;
					H_x.block(i * states_, j * states_, states_, states_) = A_p_BK_pow_[i-j-1];
					B_u.block(i * inputs_, j * inputs_, inputs_, inputs_) = K_ * A_p_BK_pow_[i-j-1] * B_estimated_;
					H_u.block(i * inputs_, j * states_, inputs_, states_) = K_ * A_p_BK_pow_[i-j-1];
				}
			}
		}
	}
	
	double hessian_matrix[horizon_ * inputs_][horizon_ * inputs_];
	double gradient_vector[horizon_ * inputs_];
	Eigen::Map<Eigen::MatrixXd> H(&hessian_matrix[0][0], horizon_ * inputs_, horizon_ * inputs_);
	Eigen::Map<Eigen::MatrixXd> g(gradient_vector, horizon_ * inputs_, 1);

	H = B_x.transpose() * Q_bar_ * B_x + B_u.transpose() * R_bar_ * B_u;
	g = B_x.transpose() * Q_bar_ * (A_x * x_measured_eigen + H_x * d_bar - x_s) + B_u.transpose() * R_bar_ * (A_u * x_measured_eigen + H_u * d_bar - u_s);

//	std::cout << A_x << " = A_x" << std::endl;//Work it!
//	std::cout << A_u << " = A_u" << std::endl;//Work it!
//	std::cout << B_x << " = B_x" << std::endl;//Work it!
//	std::cout << B_u << " = B_u" << std::endl;//Work it!
//	std::cout << d_x << " = d_x" << std::endl;
//	std::cout << d_u << " = d_u" << std::endl;
//	std::cout << u_s << " = u_s" << std::endl;//Work it!
//	std::cout << x_s << " = x_s" << std::endl;//Work it!
//	std::cout << H << " = H" << std::endl;//Work it!
//	std::cout << g << " = g" << std::endl;//Work it!
	
	// Solve the optimization problem
//	optimizer->computeOptimization();
}
