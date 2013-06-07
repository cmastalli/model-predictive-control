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
		ROS_INFO("Got param: %d", states_);
	}

	if (n_.getParam("inputs", inputs_))
	{
		//inputs_ = inputs;
		ROS_INFO("Got param: %d", inputs_);
		
	}

	if (n_.getParam("horizon", horizon_))
	{
		//horizon_ = horizon;
		ROS_INFO("Got param: %d", horizon_);
		
	}
	
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


//	Eigen::MatrixXd Qss(states_,states_);
//	Eigen::MatrixXd Pss(states_,states_);
//	Eigen::MatrixXd Rss(inputs_,inputs_);	

	//Fetch problem parameters
	double qss[states_*states_];	//TODO change to global
	
	double pss[states_*states_];
	double rss[inputs_*inputs_];

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



	//CREATION OF THE A MATRIX
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




	//CREATION OF THE B MATRIX
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

for (int i=0; i< base.size(); i++){
		std::cout<< "The "<< i <<" element of the base vector is:\n" << base[i] << std::endl;
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
	


	//CREATION OF THE Q MATRIX
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

	//CREATION OF THE R MATRIX
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

}
