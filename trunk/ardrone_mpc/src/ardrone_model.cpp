#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
#include <cmath>

#include <pluginlib/class_list_macros.h>
#include <ardrone_mpc/ardrone_model.h>

//PLUGINLIB_EXPORT_CLASS(ardrone_mpc::ArDroneModel, mpc::model::Model)

void ardrone_mpc::ArDroneModel::transformCallback(const gazebo_msgs::ModelStates& msg)
{
	position_[0] = msg.pose[2].position.x;
	position_[1] = msg.pose[2].position.y;
	position_[2] = msg.pose[2].position.z;
	q_ = msg.pose[2].orientation;

}


ardrone_mpc::ArDroneModel::ArDroneModel() 
{
	states_ = 9;
	inputs_ = 4;
	outputs_ = 9;
	position_ = new double[3];
	pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &ardrone_mpc::ArDroneModel::transformCallback, this);
}





void ardrone_mpc::ArDroneModel::computeLTIModel()
{

	/* Listen to the /tf topic and obtain the transformation
	   Convert the transformation to an Eigen object
	   Assemble the extended transformation matrix T
	   Calculate the A and B matrices of the quadrotor model
	*/ 	
	
	//tf::TransformListener listener;
	tf::StampedTransform transform;

	// This loop will check for the indicated transformation at a defined rate
	//ros::Rate listening_rate(100.0);
	//while (model_handle.ok()){
		
	
	/*try{
  		listener_.lookupTransform("/nav", "/base_stabilized	", ros::Time(0), transform);
	}
   	catch (tf::TransformException ex){
     		ROS_ERROR("%s",ex.what());
    	}

	//}
	ROS_INFO("I got the transform between /nav and /base_link frames");*/


	tf::Quaternion orientation;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(q_, orientation);
	tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);


	// Perform the transformation to an Eigen::Affine3d object
	//Eigen::Affine3d tte;
	//tf::TransformTFToEigen(transform, tte);

	Eigen::Vector3d translation;
	Eigen::Matrix3d rotation;

	rotation(0,0) = cos(yaw)*cos(pitch);
	rotation(1,0) = sin(yaw)*cos(pitch);
	rotation(2,0) = -sin(pitch);
	rotation(0,1) = (cos(yaw)*sin(pitch)*sin(roll)) - (sin(yaw)*cos(roll));
	rotation(1,1) = (sin(yaw)*sin(pitch)*sin(roll)) + (cos(yaw)*cos(roll));
	rotation(2,1) = cos(pitch)*sin(roll);
	rotation(0,2) = (cos(yaw)*sin(pitch)*cos(roll)) + (sin(yaw)*sin(roll));
	rotation(1,2) = (sin(yaw)*sin(pitch)*cos(roll)) - (cos(yaw)*sin(roll));
	rotation(2,2) = cos(pitch)*cos(roll);

	translation(0) = position_[0];
	translation(1) = position_[1];
	translation(2) = position_[2];

	// Transformation matrix to rotate the system to quadrotor standard coordinates
	Eigen::Matrix3d quad_orientation = Eigen::Matrix3d::Zero();
	quad_orientation(0,0) = 1.0;
	quad_orientation(1,1) = -1.0;
	quad_orientation(2,2) = -1.0;


	// Obtaining the translation and rotation vector from the Eigen::Affine3d object
	//translation = tte.translation();
	//rotation = tte.rotation();

	Eigen::MatrixXd T_ = Eigen::MatrixXd::Zero(states_, states_);
	T_.block<3,3>(0,0) = rotation*quad_orientation;
	T_.block<3,1>(0,8) = translation;
	T_.block<3,3>(4,4) = rotation*quad_orientation;
	T_(3,3) = -1.0;
	T_(7,7) = -1.0;
	T_(8,8) = 1.0;

	//std::cout <<"The transformation matrix T is: \n" << T_ << std::endl;
	ROS_INFO("I calculated the transformation matrix T");
		

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

	//std::cout <<"The original A matrix is: \n" << A_ << std::endl;

	// B matrix
	B_(0,0) = 0.00320;
	B_(1,1) = 0.00282;
	B_(2,2) = 0.00143;
	B_(3,3) = 0.01246;
	B_(4,0) = 0.12520;
	B_(5,1) = 0.11050;
	B_(6,2) = 0.05612;
	B_(7,3) = 0.45550;

	//std::cout <<"The original B matrix is: \n" << B_ << std::endl;
			
	// C matrix
	C_(0,0) = 1.0;
	C_(1,1) = 1.0;
	C_(2,2) = 1.0;
	C_(3,3) = 1.0;
	C_(4,4) = 1.0;
	C_(5,5) = 1.0;
	C_(6,6) = 1.0;
	C_(7,7) = 1.0;

	//std::cout <<"The original C matrix is: \n" << C_ << std::endl;

	// Now we obtain the transformed A and B matrices for the optimization problem defined by the time horizon in the MPC problem
	//Eigen::MatrixXd A_t_;
	//Eigen::MatrixXd B_t_;
	A_t_ = T_*A_*T_.inverse();
	B_t_ = T_*B_;
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
		A = A_t_;


	if (B.rows() != B_.rows()) {
		ROS_ERROR("The number of rows of the destination matrix variable and the model matrix B is different!");
		return false;
	}
	else if (B.cols() != B_.cols()) {
		ROS_ERROR("The number of columns of the destination matrix variable and the model matrix B is different!");
		return false;
	}
	else
		B = B_t_;

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

