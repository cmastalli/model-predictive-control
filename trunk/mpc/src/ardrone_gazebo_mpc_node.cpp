#include <ros/ros.h>

#include <mpc/mpc/stdmpc.h>

#include <mpc/example_models/ardrone_hovering.h>
#include <mpc/example_models/ardrone_simulator.h>
#include <mpc/optimizer/qpOASES.h>

#include <tf/transform_datatypes.h>

#include <mpc/MPCState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>

#include <realtime_tools/realtime_publisher.h>



int model_index_ = 3;
double x_meas_[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void stateCb(const gazebo_msgs::ModelStates& msg)
{
//	t_output_lin_.push_back(ros::Time::now().toSec() - t_start_);
	
    // Convert quaternion to RPY.
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(msg.pose[model_index_].orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	x_meas_[0] = msg.pose[model_index_].position.x;
	x_meas_[1] = msg.pose[model_index_].position.y;
	x_meas_[2] = msg.pose[model_index_].position.z;
	x_meas_[3] = msg.twist[model_index_].linear.x;
	x_meas_[4] = msg.twist[model_index_].linear.y;
	x_meas_[5] = msg.twist[model_index_].linear.z;
	x_meas_[6] = roll;
	x_meas_[7] = pitch;
	x_meas_[8] = yaw;
	x_meas_[9] = msg.twist[model_index_].angular.x;
	x_meas_[10] = msg.twist[model_index_].angular.y;
	x_meas_[11] = msg.twist[model_index_].angular.z;
    
//    ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "mpc");
	ros::NodeHandle nh("mpc");
	
	ros::Subscriber state_sub = nh.subscribe("gazebo/model_states", 100, stateCb);
//	ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > cmd_pub;
	cmd_pub.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist> (nh, "cmd_vel", 1));
	
	boost::scoped_ptr<realtime_tools::RealtimePublisher<mpc::MPCState> > mpc_pub;
	mpc_pub.reset(new realtime_tools::RealtimePublisher<mpc::MPCState> (nh, "data", 1));
	
	mpc_pub->lock();
	mpc_pub->msg_.states.resize(12);
	mpc_pub->msg_.reference_states.resize(12);
	mpc_pub->msg_.inputs.resize(4);
	mpc_pub->unlock();
	
	
	mpc::model::Model *model_ptr = new mpc::example_models::ArDroneHovering();
	mpc::optimizer::Optimizer *optimizer_ptr = new mpc::optimizer::qpOASES(nh);
	mpc::model::Simulator *simulator_ptr = new mpc::example_models::ArDroneSimulator();
	mpc::ModelPredictiveControl *mpc_ptr = new mpc::STDMPC(nh);
		
	mpc_ptr->resetMPC(model_ptr, optimizer_ptr, simulator_ptr);
	
	// Operation state
	double x_operation[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		
	// Set the initial conditions as the first linearization points in order to initiate the MPC algorithm
	model_ptr->setLinearizationPoints(x_operation);
	mpc_ptr->initMPC();
	
	double x_ref[12] = {0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	double *u, *delta_u, *x_bar, *u_bar, *new_state;
	double delta_xmeas[12], delta_xref[12];
//	double sampling_time = 0.0083;
	
	u = new double[4];
	new_state = new double[12];	
	x_bar = new double[12];
	u_bar = new double[4];
	
	ros::spin();
    timespec start_rt, end_rt;
    clock_gettime(CLOCK_REALTIME, &start_rt);
    int counter = 0;
	while (ros::ok()) {
		counter++;
		if (counter > 240)
			x_ref[8] = 0.50;
		if (counter > 320)
			x_ref[8] = 0.0;
		if (counter > 480)
			x_ref[0] = 1.;
		if (counter > 960)
			x_ref[1] = 1.;
				
		x_bar = model_ptr->getOperationPointsStates(); 
		for (int i = 0; i < 12; i++) {
			delta_xmeas[i] = x_meas_[i] - x_bar[i];
			delta_xref[i] = x_ref[i] - x_bar[i];
		}

		// Solving the quadratic problem to obtain the new inputs
		mpc_ptr->updateMPC(delta_xmeas, delta_xref); // Here we are also recalculating the system matrices A and B
		
		delta_u = mpc_ptr->getControlSignal();
		u_bar = model_ptr->getOperationPointsInputs();
		for (int i = 0; i < 4; i++)
			u[i] = u_bar[i] + delta_u[i];
		
		//	Publishing command to Ardrone
/*		geometry_msgs::Twist cmd;
		cmd.linear.x = u[0];
		cmd.linear.y = u[1];
		cmd.linear.z = u[2];
		cmd.angular.z = u[3];
		cmd_pub.publish(cmd);*/
		
		if (cmd_pub->trylock()) {
			cmd_pub->msg_.linear.x = u[0];
			cmd_pub->msg_.linear.y = u[1];
			cmd_pub->msg_.linear.z = u[2];
			cmd_pub->msg_.angular.z = u[3];
		}
		cmd_pub->unlockAndPublish();

		if (mpc_pub->trylock()) {
			mpc_pub->msg_.header.stamp = ros::Time::now();
			for (int j = 0; j < 12; j++) {
				mpc_pub->msg_.states[j] = x_meas_[j];
				mpc_pub->msg_.reference_states[j] = x_ref[j];
			}				
			for (int j = 0; j < 4; j++)
				mpc_pub->msg_.inputs[j] = u[j];	
		}
		mpc_pub->unlockAndPublish();
		
		
		// Shifting the state vector 
		for (int i = 0; i < 12; i++) {
			x_meas_[i] = new_state[i];
		}
	}

	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);
			
	mpc_ptr->writeToDisc();
		
	
	return 0;
}
