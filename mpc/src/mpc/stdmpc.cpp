#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <mpc/mpc/stdmpc.h>
#include <mpc/model/model.h>



/********************************************************************

NOTE: in the mapping function specified below, the size of the correspondent matrix must be entered manually, since the Map function
cannot take a variable as an argument.

*********************************************************************/



mpc::STDMPC::STDMPC(ros::NodeHandle node_handle) 
{
	model_ = 0;
	optimizer_ = 0;
	simulator_ = 0;

	// Reading of the horizon, number of constraints and number of variables from the yaml file
	if (nh_.getParam("horizon", horizon_)){	
		ROS_INFO("Got param: %d", horizon_);		
	}

	if (nh_.getParam("optimizer/number_constraints", nConst_)){	
		ROS_INFO("Got param: number of constraints = %d", nConst_);
	}

	if (nh_.getParam("optimizer/number_variables", nVar_)){
		
		if (nVar_ == horizon_*inputs_){
			ROS_INFO("Got param: number of variables = %d", nVar_);
		}

		else {
			ROS_INFO("Number of variables != Prediction Horizon x number of Inputs --> Invalid number of variables");
		}
	}


	ROS_INFO("STDMPC class successfully initialized");
}

/************************************************************************************************************
	mpc::STDMPC::resetMPC() function
************************************************************************************************************/

void mpc::STDMPC::resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator)
{

	if (model_ == 0){
		ROS_INFO("Argument model_ pointer must not be NULL");
	}

	if (optimizer_ == 0){
		ROS_INFO("Argument optimizer_ pointer must not be NULL");
	}

	if (simulator_ == 0){
		ROS_INFO("Argument simulator_ pointer must not be NULL");
	}

	model_ = model;
	optimizer_ = optimizer;
	simulator_ = simulator;

	

}

/************************************************************************************************************
	mpc::STDMPC::initMPC() function
************************************************************************************************************/

bool mpc::STDMPC::initMPC(ros::NodeHandle node_handle)
{
	nh_ = node_handle;
	// Reading of the problem variables
	states_ = model_->getStatesNumber();
	inputs_ = model_->getInputsNumber();
	outputs_ = model_->getOutputsNumber();	

	

	// Initialization of global variables 
	qss_ = new double [states_ * states_];
	pss_ = new double [states_ * states_];
	rss_ = new double [inputs_ * inputs_];

	lbA_ = new double [nConst_];
	ubA_ = new double [nConst_];

	lb_ = new double [inputs_];
	ub_ = new double [inputs_];

	lbA_bar_ = new double [nConst_ * horizon_];
	ubA_bar_ = new double [nConst_ * horizon_];

	lb_bar_ = new double [horizon_ * inputs_];
	ub_bar_ = new double [horizon_ * inputs_];

	G_bar_ = new double [horizon_ * nConst_ * horizon_ * inputs_];

	// Initialization of state space matrices
	Eigen::MatrixXd Ass(states_, states_);
	Eigen::MatrixXd Bss(states_, inputs_);
	Eigen::MatrixXd Css(outputs_, states_);

	// Obtention of the model parameters
	model_->computeDynamicModel(Ass, Bss, Css);


	// Fetch matrix Q
	XmlRpc::XmlRpcValue getQ, getR, getP;
	nh_.getParam("optimizer/states_error_weight_matrix/data", getQ);	
	ROS_ASSERT(getQ.getType() == XmlRpc::XmlRpcValue::TypeArray);
	
    
	for (int i = 0; i < getQ.size(); ++i) {
		ROS_ASSERT(getQ[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		qss_[i] = static_cast<double>(getQ[i]);
	}

	Eigen::Map<Eigen::Matrix<double,2,2,Eigen::RowMajor> > Qss(qss_, states_, states_); //TODO manually enter the size of the matrix when changing model since the Map function cannot take variables as arguments

	// Fetch matrix R
	nh_.getParam("optimizer/input_error_weight_matrix/data", getR);	
	ROS_ASSERT(getR.getType() == XmlRpc::XmlRpcValue::TypeArray);
	
    
	for (int i = 0; i < getR.size(); ++i) {
		ROS_ASSERT(getR[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		rss_[i] = static_cast<double>(getR[i]);
	}

	Eigen::Map<Eigen::Matrix<double,1,1,Eigen::RowMajor> > Rss(rss_, inputs_, inputs_); //TODO manually enter the size of the matrix when changing model since the Map function cannot take variables as arguments

	// Fetch matrix P
	nh_.getParam("optimizer/terminal_state_weight_matrix/data", getP);	
	ROS_ASSERT(getP.getType() == XmlRpc::XmlRpcValue::TypeArray);
	
    
	for (int i = 0; i < getP.size(); ++i) {
		ROS_ASSERT(getP[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		pss_[i] = static_cast<double>(getP[i]);
	}

	Eigen::Map<Eigen::Matrix<double,2,2,Eigen::RowMajor> > Pss(pss_, states_, states_); //TODO manually enter the size of the matrix when changing model since the Map function cannot take variables as arguments

	
	
	std::cout << Qss <<" = Qss" << std::endl;
	std::cout << Pss <<" = Pss" << std::endl;
	std::cout << Rss <<" = Rss" << std::endl;



	//CREATION OF THE A MATRIX
	static Eigen::MatrixXd A((horizon_ + 1) * states_, states_);
	Eigen::MatrixXd An(states_, states_);

    //Assign the identity matrix to the first position of the matrix A
    A.block(0, 0, states_, states_) = Eigen::MatrixXd::Identity(states_, states_); 

    //Loop to fill the A matrix with the corresponding power of Ass
	Eigen::MatrixXd temp_1(states_,states_);
	temp_1 = Eigen::MatrixXd::Identity(states_,states_);
    for (int i = 1; i <= horizon_; i++) {
		// Loop to raise the current matrix to the j-th power
		An = Ass;	
		temp_1 = An * temp_1;
		An = temp_1;
	
		A.block(i*states_,0,states_,states_) = An;
    }

	A_bar_ = A; 
	std::cout << A_bar_ << " = A_bar" << std::endl;	

	//CREATION OF THE B MATRIX
	static Eigen::MatrixXd B = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, horizon_ * inputs_); 
	Eigen::MatrixXd Bn(states_,states_);
	Eigen::MatrixXd temp_2(states_,states_);
	temp_2 = Eigen::MatrixXd::Identity(states_,states_);

	//Creation of the standard vector to define the first row
	std::vector<Eigen::MatrixXd> base;
	base.push_back(Eigen::MatrixXd::Zero(states_,inputs_));
	base.push_back(Bss);


	//CREATING THE BASE VECTOR
	Eigen::MatrixXd aux(states_,inputs_);
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
			B.block(k*states_, j*inputs_, states_, inputs_) = base[z];
			z++;
		}
	}
	B_bar_ = B; 
	std::cout << B_bar_ << " = B_bar" << std::endl;
	


	//CREATION OF THE Q MATRIX
	Eigen::MatrixXd Q((horizon_ + 1) * states_, (horizon_ + 1) * states_);

	for (int q = 0; q < horizon_ + 1; q++) {
		for (int qq = 0; qq < horizon_ + 1; qq++) {
			if (q == qq)
				Q.block(q * states_, qq * states_, states_, states_) = Qss;
			else
				Q.block(q * states_, qq * states_, states_, states_) = Eigen::MatrixXd::Zero(states_, states_);
		}
	} // TODO add the P matrix to the last position of the Q matrix

	Q_bar_ = Q; 
	std::cout << Q_bar_ << " = Q_bar" << std::endl;

	//CREATION OF THE R MATRIX
	Eigen::MatrixXd R(horizon_ * inputs_, horizon_ * inputs_);

	for (int r = 0; r < horizon_; r++) {
		for (int rr = 0; rr < horizon_; rr++) {
			if (r == rr)
				R.block(r * inputs_, rr * inputs_, inputs_, inputs_) = Rss;
			else
				R.block(r * inputs_, rr * inputs_, inputs_, inputs_) = Eigen::MatrixXd::Zero(inputs_, inputs_);
		}
	}

	std::cout << R << " = R_bar" << std::endl;

	// Initialization of the array and mapping into an Eigen object
	H_bar_ = new double [horizon_ * inputs_ * horizon_ * inputs_];
	Eigen::Map<Eigen::Matrix<double,5,5,Eigen::RowMajor> > H_bar(H_bar_, horizon_*inputs_, horizon_*inputs_);
	
	// Computing the values of the Hessian matrix
	H_bar = B.transpose() * Q * B + R;	
	std::cout << H_bar << " = H_bar" << std::endl;
	
	
/*	for (int t = 0; t < H.rows() * H.cols(); t++) {
		H_array[t] = *H_ptr;
		H_ptr++;	
	} 

	for (int i = 0; i < horizon_ * inputs_ * horizon_ * inputs_; i++) {
		std::cout<< "H[" << i << "] = "<< H_array[i] << std::endl;
	}
*/
	
	return true;
}

/************************************************************************************************************
	mpc::STDMPC::updateMPC() function
************************************************************************************************************/


void mpc::STDMPC::updateMPC(Eigen::MatrixXd x_k, Eigen::MatrixXd x_ref)
{

	// Constructing the extended reference vector
	Eigen::MatrixXd x_ref_bar(states_ * (horizon_ + 1), 1);	
	for (int i = 0; i < horizon_ + 1; i++) {
		x_ref_bar.block(states_ * i, 0, states_, 1) = x_ref;
	}
	std::cout << x_ref_bar << " = x_ref_bar" << std::endl;

	// Constructing the gradient vector g
	g_ = new double [horizon_ * inputs_ * 1];
	Eigen::Map<Eigen::VectorXd> G_(g_, horizon_*inputs_);	


	G_ = B_bar_.transpose() * Q_bar_ * A_bar_ * x_k - B_bar_.transpose() * Q_bar_ * x_ref_bar;
	std::cout << G_ << " = g" << std::endl;
	
/*	for (int v = 0; v < g.rows() * g.cols(); v++){
		g_array[v] = *g_ptr;
		g_ptr++;
	}
*/
	



	/******************************************************************************
	CREATION OF THE EXTENDED VARIABLES FOR CONSTRAINT HANDLING
	******************************************************************************/

	// Reading the constraint variables from the configuration file and mapping them to Eigen objects

	// Reading the constraint vectors
	XmlRpc::XmlRpcValue getlowX, getuppX;
	nh_.getParam("optimizer/constraints/constraint_vector_low", getlowX);    
	ROS_ASSERT(getlowX.getType() == XmlRpc::XmlRpcValue::TypeArray);
       
	nh_.getParam("optimizer/constraints/constraint_vector_upp", getuppX);        
	ROS_ASSERT(getuppX.getType() == XmlRpc::XmlRpcValue::TypeArray);

	
	if (getuppX.size() == getlowX.size()) {
		for (int i = 0; i < getuppX.size(); ++i) {
			ROS_ASSERT(getlowX[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			lbA_[i] = static_cast<double>(getlowX[i]);

			ROS_ASSERT(getuppX[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			ubA_[i] = static_cast<double>(getuppX[i]);
		}

	}
       
	// Mapping the readed arrays into Eigen objects
	Eigen::Map<Eigen::VectorXd> LbA(lbA_,nConst_);
	Eigen::Map<Eigen::VectorXd> UbA(ubA_,nConst_);

	
	// Reading the bound vectors
	XmlRpc::XmlRpcValue getlowU, getuppU;
	nh_.getParam("optimizer/constraints/bound_vector_low", getlowU);
	ROS_ASSERT(getlowU.getType() == XmlRpc::XmlRpcValue::TypeArray);

	nh_.getParam("optimizer/constraints/bound_vector_upp", getuppU);
	ROS_ASSERT(getuppU.getType() == XmlRpc::XmlRpcValue::TypeArray);
       
	if (getuppU.size() == getlowU.size()) {
		for (int i = 0; i < getuppU.size(); ++i) {
			ROS_ASSERT(getlowU[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			lb_[i] = static_cast<double>(getlowU[i]);                              

			ROS_ASSERT(getuppU[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			ub_[i] = static_cast<double>(getuppU[i]);
		}
	}
       
	// Mapping the readed arrays into Eigen objects
	Eigen::Map<Eigen::VectorXd> Lb(lb_, inputs_);
	Eigen::Map<Eigen::VectorXd> Ub(ub_, inputs_);

	double m[nConst_ * states_];
       
	XmlRpc::XmlRpcValue getmatrixM;
	nh_.getParam("optimizer/constraints/constraint_matrix_M", getmatrixM);  
	ROS_ASSERT(getmatrixM.getType() == XmlRpc::XmlRpcValue::TypeArray);
       
   
	for (int i = 0; i < getmatrixM.size(); ++i) {
		ROS_ASSERT(getmatrixM[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		m[i] = static_cast<double>(getmatrixM[i]);
	}
	ROS_INFO("nConst= %d states = %d", nConst_, states_);
	Eigen::Map<Eigen::Matrix<double,3,2,Eigen::RowMajor> > M(m, nConst_, states_);

	
	std::cout << LbA << " = LbA" << std::endl;
	std::cout << UbA << " = UbA" << std::endl;
	std::cout << Ub << " = Lb" << std::endl;
	std::cout << Lb << " = Ub" << std::endl;
	std::cout << M << " = M" << std::endl;

	Eigen::Map<Eigen::VectorXd> LbA_bar(lbA_bar_,nConst_*horizon_);
	Eigen::Map<Eigen::VectorXd> UbA_bar(ubA_bar_,nConst_*horizon_);

	Eigen::Map<Eigen::VectorXd> Lb_bar(lb_bar_,inputs_*horizon_);
	Eigen::Map<Eigen::VectorXd> Ub_bar(ub_bar_,inputs_*horizon_);

	for (int i = 0; i < horizon_; i++) {
		LbA_bar.block(i * nConst_, 0, nConst_, 1) = LbA;
		UbA_bar.block(i * nConst_, 0, nConst_, 1) = UbA;

		Lb_bar.block(i * inputs_, 0, inputs_, 1) = Lb;
		Ub_bar.block(i * inputs_, 0, inputs_, 1) = Ub;
	}

	std::cout << LbA_bar << " = LbA_bar" << std::endl;
	std::cout << UbA_bar << " = UbA_bar" << std::endl;

	std::cout << Lb_bar << " = Lb_bar" << std::endl;
	std::cout << Ub_bar << " = Ub_bar" << std::endl;
 
   

	// Creation of the extended constraint matrix G_bar_
	Eigen::MatrixXd M_bar = Eigen::MatrixXd::Zero(nConst_*horizon_ , (horizon_ + 1)*states_);

	for (int i = 0; i < horizon_; i++) {
		for (int j = 0; j < horizon_ + 1; j++) {
			if (i == j) {
				M_bar.block(i * nConst_,j * states_, nConst_, states_) = M;
			}
		}
	}      

	//std::cout << M_bar << " = M_bar" << std::endl;

	LbA_bar = LbA_bar - M_bar * A_bar_ * x_k;
	UbA_bar = UbA_bar - M_bar * A_bar_ * x_k;
       
	// Mapping of the extended constraint matrix G_bar_
	Eigen::Map<Eigen::Matrix<double, 15, 5, Eigen::RowMajor> > G_bar(G_bar_, nConst_ * horizon_, horizon_ * inputs_);
       
	G_bar = M_bar * B_bar_;

	std::cout << G_bar << " = G_bar" << std::endl;
        
}
