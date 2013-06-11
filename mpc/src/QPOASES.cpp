#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <mpc/optimizer/QPOASES.h>
#include <mpc/model/model.h>
#include <mpc/optimizer/optimizer.h>

/********************************************************************

NOTE: in the mapping function specified below, the size of the correspondent matrix must be entered manually, since the Map function
cannot take a variable as an argument.

*********************************************************************/



mpc::optimizer::QPOASES::QPOASES(ros::NodeHandle node, mpc::model::Model *model_ptr) : n_(node)
{
	model_ = model_ptr;

	

	//int states, inputs, horizon;
	if (n_.getParam("states", states_))
	{
		//states_ = states;
		ROS_INFO("Got param: number of states = %d", states_);
	}

	if (n_.getParam("inputs", inputs_))
	{
		//inputs_ = inputs;
		ROS_INFO("Got param: number of inputs = %d", inputs_);
		
	}

	if (n_.getParam("horizon", horizon_))
	{
		//horizon_ = horizon;
		ROS_INFO("Got param: prediction horizon = %d", horizon_);
		
	}

	if (n_.getParam("optimizer/number_constraints", nConst_))
	{
		//horizon_ = horizon;
		ROS_INFO("Got param: number of constraints = %d", nConst_);
		
	}

	if (n_.getParam("optimizer/number_variables", nVar_))
	{
		if (nVar_ == horizon_*inputs_){
			ROS_INFO("Got param: number of variables = %d", nVar_);
		}
		else {
			ROS_INFO("Number of variables != Prediction Horizon x number of Inputs --> Invalid number of variables");
		}
	}

	qss = new double [states_*states_];
	pss = new double [states_*states_];
	rss = new double [inputs_*inputs_];

	lbA_ = new double [nConst_];
	ubA_ = new double [nConst_];

	lb_ = new double [inputs_];
	ub_ = new double [inputs_];

	lbA_bar_ = new double [nConst_*horizon_];
	ubA_bar_ = new double [nConst_*horizon_];

	lb_bar_ = new double [horizon_*inputs_];
	ub_bar_ = new double [horizon_*inputs_];

	G_bar_ = new double [horizon_*nConst_*horizon_*inputs_];
	
	ROS_INFO("QPOASES class successfully initialized");
}




// TODO Define a function to read from a .yaml file to get nWSR and cputime variables

void mpc::optimizer::QPOASES::computeMPC(Eigen::VectorXd x_k, Eigen::VectorXd x_ref)
{
	Eigen::MatrixXd Ass(states_,states_);
	Eigen::MatrixXd Bss(states_,inputs_);

	// Obtention of the model parameters
	model_->getModelParameterA(Ass);
	model_->getModelParameterB(Bss);
	

	//Fetch problem parameters

	// Fetch matrix Q
	XmlRpc::XmlRpcValue getQ, getR, getP;
	n_.getParam("optimizer/states_error_weight_matrix/data", getQ);	
	ROS_ASSERT(getQ.getType() == XmlRpc::XmlRpcValue::TypeArray);
	
    
	for (int i = 0; i < getQ.size(); ++i){
			
			ROS_ASSERT(getQ[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			qss[i] = static_cast<double>(getQ[i]);
	}

	Eigen::Map<Eigen::Matrix<double,2,2,Eigen::RowMajor> > Qss(qss,states_,states_); //TODO manually enter the size of the matrix when changing model since the Map function cannot take variables as arguments

	// Fetch matrix R
	n_.getParam("optimizer/input_error_weight_matrix/data", getR);	
	ROS_ASSERT(getR.getType() == XmlRpc::XmlRpcValue::TypeArray);
	
    
	for (int i = 0; i < getR.size(); ++i){
			
			ROS_ASSERT(getR[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			rss[i] = static_cast<double>(getR[i]);
	}

	Eigen::Map<Eigen::Matrix<double,1,1,Eigen::RowMajor> > Rss(rss,inputs_,inputs_); //TODO manually enter the size of the matrix when changing model since the Map function cannot take variables as arguments

	// Fetch matrix P
	n_.getParam("optimizer/terminal_state_weight_matrix/data", getP);	
	ROS_ASSERT(getP.getType() == XmlRpc::XmlRpcValue::TypeArray);
	
    
	for (int i = 0; i < getP.size(); ++i){
			
			ROS_ASSERT(getP[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			pss[i] = static_cast<double>(getP[i]);
	}

	Eigen::Map<Eigen::Matrix<double,2,2,Eigen::RowMajor> > Pss(pss,states_,states_); //TODO manually enter the size of the matrix when changing model since the Map function cannot take variables as arguments

	
	


	std::cout << Qss <<" = Qss" << std::endl;
	std::cout << Pss <<" = Pss" << std::endl;
	std::cout << Rss <<" = Rss" << std::endl;



/******************************************************************************
CREATION OF THE A MATRIX
******************************************************************************/

	Eigen::MatrixXd A((horizon_ + 1) * states_, states_);
	Eigen::MatrixXd An(states_, states_);

    //Assign the identity matrix to the first position of the matrix A
    A.block(0,0,states_,states_) = Eigen::MatrixXd::Identity(states_,states_); 

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




/******************************************************************************
CREATION OF THE B MATRIX
******************************************************************************/

	Eigen::MatrixXd B = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, horizon_ * inputs_); 
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
	std::cout << B << " = B_bar" << std::endl;
	


/******************************************************************************
CREATION OF THE Q MATRIX
******************************************************************************/

	Eigen::MatrixXd Q((horizon_ + 1) * states_, (horizon_ + 1) * states_);

	for (int q = 0; q < horizon_ + 1; q++) {
		for (int qq = 0; qq < horizon_ + 1; qq++) {
			if (q == qq)
				Q.block(q*states_,qq*states_,states_,states_) = Qss;
			else
				Q.block(q*states_,qq*states_,states_,states_) = Eigen::MatrixXd::Zero(states_,states_);
		}
	} // TODO add the P matrix to the last position of the Q matrix

	std::cout << Q << " = Q_bar" << std::endl;

/******************************************************************************
CREATION OF THE R MATRIX
******************************************************************************/

	Eigen::MatrixXd R(horizon_ * inputs_, horizon_ * inputs_);

	for (int r = 0; r < horizon_; r++) {
		for (int rr = 0; rr < horizon_; rr++) {
			if (r == rr)
				R.block(r*inputs_,rr*inputs_,inputs_,inputs_) = Rss;
			else
				R.block(r*inputs_,rr*inputs_,inputs_,inputs_) = Eigen::MatrixXd::Zero(inputs_,inputs_);
		}
	}

	std::cout << R << " = R_bar" << std::endl;

	// Creation of the H and g matrices and assignment to a standard C array
	Eigen::MatrixXd H(horizon_ * inputs_, horizon_ * inputs_);
	H = B.transpose()*Q*B + R;
	std::cout << H << " = H" << std::endl;

	Eigen::MatrixXd g(horizon_ * inputs_, 1);

	Eigen::MatrixXd x_ref_bar(states_ * (horizon_ + 1), 1);	
	for (int i=0; i<(horizon_ + 1); i++){
		x_ref_bar.block(states_*i, 0, states_, 1) = x_ref;
	}
	std::cout << x_ref_bar << " = x_ref_bar" << std::endl;

	g = B.transpose()*Q*A*x_k - B.transpose()*Q*x_ref_bar;
	std::cout << g << " = g" << std::endl;


	double * H_ptr;
	double * g_ptr;

	H_ptr = H.data();
	g_ptr = g.data();

	double H_array[horizon_*inputs_*horizon_*inputs_];
	double g_array[horizon_*inputs_];	

	for (int t = 0; t < (H.rows() * H.cols()); t++) {
		H_array[t] = *H_ptr;
		H_ptr++;	
	}

	for (int v = 0; v < (g.rows() * g.cols()); v++){
		g_array[v] = *g_ptr;
		g_ptr++;
	}

	for (int i = 0; i < horizon_ * inputs_ * horizon_ * inputs_; i++) {
		std::cout<< "H[" << i << "] = "<< H_array[i] << std::endl;
	}

	for (int j = 0; j < horizon_ * inputs_; j++) {
		std::cout<< "g[" << j << "] = "<< g_array[j] << std::endl;
	}

/****************************************************************************** 
CREATION OF THE EXTENDED VARIABLES FOR CONSTRAINT HANDLING 
******************************************************************************/

// Reading the constraint variables from the configuration file and mapping them to Eigen objects

	// Reading the constraint vectors
	XmlRpc::XmlRpcValue getlowX, getuppX;
	n_.getParam("optimizer/constraints/constraint_vector_low", getlowX);	
	ROS_ASSERT(getlowX.getType() == XmlRpc::XmlRpcValue::TypeArray);
	
    n_.getParam("optimizer/constraints/constraint_vector_upp", getuppX);	
	ROS_ASSERT(getuppX.getType() == XmlRpc::XmlRpcValue::TypeArray);


	if (getuppX.size() == getlowX.size()){
		for (int i = 0; i < getuppX.size(); ++i){
				
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
	n_.getParam("optimizer/constraints/bound_vector_low", getlowU);	
	ROS_ASSERT(getlowU.getType() == XmlRpc::XmlRpcValue::TypeArray);

	n_.getParam("optimizer/constraints/bound_vector_upp", getuppU);	
	ROS_ASSERT(getuppU.getType() == XmlRpc::XmlRpcValue::TypeArray);
	
    if (getuppU.size() == getlowU.size()){
		for (int i = 0; i < getuppU.size(); ++i){
			
				ROS_ASSERT(getlowU[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				lb_[i] = static_cast<double>(getlowU[i]);				

				ROS_ASSERT(getuppU[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				ub_[i] = static_cast<double>(getuppU[i]);
		}
	}
	
	// Mapping the readed arrays into Eigen objects
	Eigen::Map<Eigen::VectorXd> Lb(lb_,inputs_);
	Eigen::Map<Eigen::VectorXd> Ub(ub_,inputs_);

	double m[nConst_*states_];
	
	XmlRpc::XmlRpcValue getmatrixM;
	n_.getParam("optimizer/constraints/constraint_matrix_M", getmatrixM);	
	ROS_ASSERT(getmatrixM.getType() == XmlRpc::XmlRpcValue::TypeArray);
	
    
	for (int i = 0; i < getmatrixM.size(); ++i){
			
			ROS_ASSERT(getmatrixM[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			m[i] = static_cast<double>(getmatrixM[i]);
	}

	Eigen::Map<Eigen::Matrix<double,3,2,Eigen::RowMajor> > M(m, nConst_, states_);

/*
	std::cout << LbA << " = LbA" << std::endl;
	std::cout << UbA << " = UbA" << std::endl;
	std::cout << Ub << " = Lb" << std::endl;
	std::cout << Lb << " = Ub" << std::endl;
	std::cout << M << " = M" << std::endl;
*/
	Eigen::Map<Eigen::VectorXd> LbA_bar(lbA_bar_,nConst_*horizon_);
	Eigen::Map<Eigen::VectorXd> UbA_bar(ubA_bar_,nConst_*horizon_);

	Eigen::Map<Eigen::VectorXd> Lb_bar(lb_bar_,inputs_*horizon_);
	Eigen::Map<Eigen::VectorXd> Ub_bar(ub_bar_,inputs_*horizon_);

	for (int i=0; i<horizon_; i++){
		
		LbA_bar.block(i*nConst_, 0, nConst_, 1) = LbA;
		UbA_bar.block(i*nConst_, 0, nConst_, 1) = UbA;

		Lb_bar.block(i*inputs_, 0, inputs_, 1) = Lb;
		Ub_bar.block(i*inputs_, 0, inputs_, 1) = Ub;
	}

	std::cout << LbA_bar << " = LbA_bar" << std::endl;
	std::cout << UbA_bar << " = UbA_bar" << std::endl;

	std::cout << Lb_bar << " = Lb_bar" << std::endl;
	std::cout << Ub_bar << " = Ub_bar" << std::endl;
  
	

	// Creation of the extended constraint matrix G_bar_
	Eigen::MatrixXd M_bar = Eigen::MatrixXd::Zero(nConst_*horizon_ , (horizon_ + 1)*states_);

	for (int i = 0; i < horizon_; i++) {
		for (int j = 0; j < horizon_ + 1; j++) {
			if (i == j){
				M_bar.block(i*nConst_,j*states_,nConst_,states_) = M;
			}
		}
	}	

	//std::cout << M_bar << " = M_bar" << std::endl;

	LbA_bar = LbA_bar - M_bar*A*x_k;
	UbA_bar = UbA_bar - M_bar*A*x_k;
	
	// Mapping of the extended constraint matrix G_bar_
	Eigen::Map<Eigen::Matrix<double, 15, 5, Eigen::RowMajor> > G_bar(G_bar_, nConst_*horizon_, horizon_*inputs_);
	
	G_bar = M_bar*B;

	std::cout << G_bar << " = G_bar" << std::endl;
	
}
