#include <ros/ros.h>
#include <iostream>

#include <ardrone_mpc/ardrone_model.h>


ardrone_mpc::ArDroneModel::ArDroneModel() 
{
	states_ = 9;
	inputs_ = 4;
	outputs_ = 9;
}





void ardrone_mpc::ArDroneModel::computeLTIModel()
{

	/* Listen to the /tf topic and obtain the transformation
	   Convert the transformation to an Eigen object
	   Assemble the extended transformation matrix T
	   Calculate the A and B matrices of the quadrotor model
	*/ 	
	ros::NodeHandle model_handle;
	tf::TransformListener listener;

	// This loop will check for the indicated transformation at a defined rate
	ros::Rate listening_rate(100.0);
	while (model_handle.ok()){
		
		tf::StampedTransform transform;
    	try{
      		listener.lookupTransform("/nav", "/base_link", ros::Time(0), transform);
    	}
    	catch (tf::TransformException ex){
      		ROS_ERROR("%s",ex.what());
    	}

	}

	// Perform the transformation to an Eigen::Affine3d object
	Eigen::Affine3d tte;
	tf::transformTFToEigen(transform, tte);

	Eigen::Vector3d translation;
	Eigen::Matrix3d rotation;
	// Obtaining the translation and rotation vector from the Eigen::Affine3d object
	translation = tte.translation();
	rotation = tte.rotation();

	Eigen::MatrixXd T_ = Eigen::MatrixXd::Zero(states_, states_);
	T_.block<3,3>(0,0) = rotation;
	T_.block<3,1>(0,8) = translation;
	T_.block<3,3>(4,4) = rotation;
	T_(3,3) = -1.0;
	T_(7,7) = -1.0;
	T_(8,8) = 1.0;

	std::cout <<"The transformation matrix T is: \n" << T_ << std::endl;
	
		

	A_ = Eigen::MatrixXd::Zero(states_, states_);
	B_ = Eigen::MatrixXd::Zero(states_, inputs_);
	C_ = Eigen::MatrixXd::Zero(outputs_, states_);

	// A matrix
	A_(0,0) = 1.0;
	A_(0,1) = 0.0468;
	A_(1,2) = 1.0;
	A_(1,3) = 0.0470;
	A_(2,4) = 1.0;
	A_(2,5) = 0.0471;
	A_(3,6) = 1.0;
	A_(3,7) = 0.0381;
	A_(8,8) = 1.0;
	A_(4,1) = 0.8727;
	A_(5,3) = 0.8825;
	A_(6,5) = 0.8878;
	A_(7,7) = 0.5662;

	std::cout <<"The original A matrix is: \n" << A_ << std::endl;

	// B matrix
	B_(0,0) = 0.00320;
	B_(1,1) = 0.00282;
	B_(2,2) = 0.00143;
	B_(3,3) = 0.01246;
	
			
	// C matrix
	C_(0,0) = 0.0000;
	C_(0,1) = 1.0000;
}

bool ardrone_mpc::ArDroneModel::computeDynamicModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C)
{
	computeLTIModel();

	if (A.rows() != A_.rows()) {
		ROS_ERROR("The number of rows of the destination matrix variable and the model matrix A is different!");
		return false;
	}
	else if (A.cols() != A_.cols()) {
		ROS_ERROR("The number of columns of the destination matrix variable and the model matrix A is different!");
		return false;
	}
	else
		A = A_;


	if (B.rows() != B_.rows()) {
		ROS_ERROR("The number of rows of the destination matrix variable and the model matrix B is different!");
		return false;
	}
	else if (B.cols() != B_.cols()) {
		ROS_ERROR("The number of columns of the destination matrix variable and the model matrix B is different!");
		return false;
	}
	else
		B = B_;

	if (C.rows() != C_.rows()) {
		ROS_ERROR("The number of rows of the destination matrix variable and the model matrix C is different!");
		return false;
	}
	else if (C.cols() != C_.cols()) {
		ROS_ERROR("The number of columns of the destination matrix variable and the model matrix C is different!");
		return false;
	}
	else
		C = C_;


	return true;
}

