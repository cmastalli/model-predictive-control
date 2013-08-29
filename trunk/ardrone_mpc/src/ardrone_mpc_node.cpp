#include <ros/ros.h>

#include <mpc/mpc/stdmpc.h>

#include <ardrone_mpc/ardrone_sim.h>
#include <ardrone_mpc/ardrone_model.h>
#include <mpc/optimizer/qpOASES.h>

#include <mpc/MPCState.h>
#include <ardrone_mpc/Reference.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3.h>
//#include <math/gzmath.hh>
//#include <LinearMath/btMatrix3x3.h>
//#include <msgs/msgs.hh>
#include <gazebo_msgs/ModelStates.h>

#include <realtime_tools/realtime_publisher.h>

// Global variables

	double x_ref_[9];
	double x_meas_[9]; 

void refCallback(const ardrone_mpc::Reference& msg)
{
	x_ref_[0] = msg.reference_states[0];
	x_ref_[1] = msg.reference_states[1];
	x_ref_[2] = msg.reference_states[2];
	x_ref_[3] = msg.reference_states[3];
	x_ref_[4] = msg.reference_states[4];
	x_ref_[5] = msg.reference_states[5];
	x_ref_[6] = msg.reference_states[6];
	x_ref_[7] = msg.reference_states[7];
	x_ref_[8] = msg.reference_states[8];
}

void stateCallback(const gazebo_msgs::ModelStates& msg)
{
	x_meas_[0] = msg.pose[2].position.x;
	x_meas_[1] = msg.pose[2].position.y;
	x_meas_[2] = msg.pose[2].position.z;
	x_meas_[4] = msg.twist[2].linear.x;
	x_meas_[5] = msg.twist[2].linear.y;
	x_meas_[6] = msg.twist[2].linear.z;
	x_meas_[7] = msg.twist[2].angular.z;

	// Obtain the orientation of the quadrotor as a quaternion and transform it into Euler angles
	//gazebo::math::Quaternion q = msg.pose[2].orientation;
//	gazebo::math::Quaternion q = gazebo::msgs::Convert(msg.pose[2].orientation);

//	gazebo::math::Quaternion ok = Quaternion(q);


//	gazebo::math::Vector3 RPY = q.GetAsEuler();

	tf::Quaternion q;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(msg.pose[2].orientation, q);
	//btMatrix3x3(q).getEulerYPR(yaw, pitch, roll);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	x_meas_[3] = yaw;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ardrone_mpc");
	ros::NodeHandle node_handle("ardrone_mpc");
	boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > mpc_pub;
	mpc_pub.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist> (node_handle, "/cmd_vel", 1));

	ros::Subscriber reference_subscriber = node_handle.subscribe("/ardrone_mpc/reference", 1, refCallback);

	/** Get the initial condition of the quadrotor **/
	ros::Subscriber state_subscriber = node_handle.subscribe("/gazebo/model_states", 1, stateCallback);

	
	
	mpc::model::Model *model_ptr = new ardrone_mpc::ArDroneModel();
	mpc::optimizer::Optimizer *optimizer_ptr = new mpc::optimizer::qpOASES(node_handle);
	mpc::model::Simulator *simulator_ptr = new ardrone_mpc::ArDroneSimulator();
	
	mpc::ModelPredictiveControl *mpc_ptr = new mpc::STDMPC(node_handle);
	
	
	mpc_ptr->resetMPC(model_ptr, optimizer_ptr, simulator_ptr);
	mpc_ptr->initMPC();
	

	
	
	//double sampling_time = 0.01;
	double *control_signal;
	
	//double *new_state;
	
	//ros::Rate control_loop_rate(20);	
    timespec start_rt, end_rt;
    clock_gettime(CLOCK_REALTIME, &start_rt);
	while (ros::ok()) {
	//for (int i= 0; i < 5; i++){
				
		/** Calculate the control signal for the system **/
		mpc_ptr->updateMPC(x_meas_, x_ref_);
		control_signal = mpc_ptr->getControlSignal();
		//new_state = simulator_ptr->simulatePlant(x_meas, control_signal, sampling_time);
		

		/** Send the control signal to Gazebo **/
		
		//x_meas[0] = new_state[0];
		//x_meas[1] = new_state[1];
		
		if (mpc_pub->trylock()) {
			//mpc_pub->msg_.header.stamp = ros::Time::now();
			/*mpc_pub->msg_.states[0] = x_meas[0];
			mpc_pub->msg_.states[1] = x_meas[1];
			mpc_pub->msg_.reference_states[0] = x_ref[0];
			mpc_pub->msg_.reference_states[1] = x_ref[1];*/
			mpc_pub->msg_.linear.x = control_signal[0];
			mpc_pub->msg_.linear.y = control_signal[1];
			mpc_pub->msg_.linear.z = control_signal[2];
			mpc_pub->msg_.angular.z = control_signal[3];
		}
		mpc_pub->unlockAndPublish();
		
		if (!ros::ok()) {			
			clock_gettime(CLOCK_REALTIME, &end_rt);
			double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
			ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);
			
			mpc_ptr->writeToDisc();
		}
		ROS_INFO("1sp");
		ros::spinOnce();
		ROS_INFO("2sp");
		//control_loop_rate.sleep();
	}
	
	
	return 0;
}

