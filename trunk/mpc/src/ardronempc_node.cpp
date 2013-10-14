#include <ros/ros.h>

#include <mpc/mpc/stdmpc.h>

#include <mpc/example_models/ardrone.h>
#include <mpc/example_models/ardrone_simulator.h>
#include <mpc/optimizer/qpOASES.h>

#include <mpc/MPCState.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>

#include <realtime_tools/realtime_publisher.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "mpc");
	ros::NodeHandle node_handle("mpc");
	boost::scoped_ptr<realtime_tools::RealtimePublisher<mpc::MPCState> > mpc_pub;
	mpc_pub.reset(new realtime_tools::RealtimePublisher<mpc::MPCState> (node_handle, "data", 1));
	mpc_pub->lock();
	mpc_pub->msg_.states.resize(12);
	mpc_pub->msg_.reference_states.resize(12);
	mpc_pub->msg_.inputs.resize(4);
	mpc_pub->unlock();
	
	
	mpc::model::Model *model_ptr = new mpc::example_models::ArDrone();
	mpc::optimizer::Optimizer *optimizer_ptr = new mpc::optimizer::qpOASES(node_handle);
	mpc::model::Simulator *simulator_ptr = new mpc::example_models::ArDroneSimulator();
	
	mpc::ModelPredictiveControl *mpc_ptr = new mpc::STDMPC(node_handle);
	
	
	mpc_ptr->resetMPC(model_ptr, optimizer_ptr, simulator_ptr);
	mpc_ptr->initMPC();
	
	
	double x_ref[12] = {0., 0., 10000000000.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.};
	double x_meas[12] = {0., 0.,  0.3, 0., 0., 0., 0., 0., 0., 0., 0., 0.};

	// linearization points
	double x_operation[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	//double u_operation[4] = {360.0, 360.0, 360.0, 360.0};

	 
																																																																																																																																																																																																																																																				
	double *control_signal;
	double sampling_time;
	double *new_state;	
		
    timespec start_rt, end_rt;
    clock_gettime(CLOCK_REALTIME, &start_rt);
	while (ros::ok()) {
		//model_ptr->setStates(x_meas);
		//model_ptr->setInputs(u_operation); 

		double x_lin_meas[12];
		x_lin_meas[0] = x_meas[0] - x_operation[0];
		x_lin_meas[1] = x_meas[1] - x_operation[1];
		x_lin_meas[2] = x_meas[2] - x_operation[2];
		x_lin_meas[3] = x_meas[3] - x_operation[3];
		x_lin_meas[4] = x_meas[4] - x_operation[4];
		x_lin_meas[5] = x_meas[5] - x_operation[5];
		x_lin_meas[6] = x_meas[6] - x_operation[6];
		x_lin_meas[7] = x_meas[7] - x_operation[7];
		x_lin_meas[8] = x_meas[8] - x_operation[8];
		x_lin_meas[9] = x_meas[9] - x_operation[9];
		x_lin_meas[10] = x_meas[10] - x_operation[10];	
		x_lin_meas[11] = x_meas[11] - x_operation[11]; 

		double x_lin_ref[12];
		x_lin_ref[0] = x_ref[0] - x_operation[0];
		x_lin_ref[1] = x_ref[1] - x_operation[1];
		x_lin_ref[2] = x_ref[2] - x_operation[2];
		x_lin_ref[3] = x_ref[3] - x_operation[3];
		x_lin_ref[4] = x_ref[4] - x_operation[4];
		x_lin_ref[5] = x_ref[5] - x_operation[5];
		x_lin_ref[6] = x_ref[6] - x_operation[6];
		x_lin_ref[7] = x_ref[7] - x_operation[7];
		x_lin_ref[8] = x_ref[8] - x_operation[8];
		x_lin_ref[9] = x_ref[9] - x_operation[9];
		x_lin_ref[10] = x_ref[10] - x_operation[10];	
		x_lin_ref[11] = x_ref[11] - x_operation[11];
		
		mpc_ptr->updateMPC(x_lin_meas, x_lin_ref); // Here we are also recalculating the system matrices A and B
		ROS_INFO("u=[%f,%f,%f,%f]", control_signal[0], control_signal[1], control_signal[2], control_signal[3]);
		control_signal = mpc_ptr->getControlSignal();

		// Getting the A and B matrices to compute the linearization point of the control signal for the steady state at the current sample
		Eigen::MatrixXd A(12,12);
		Eigen::MatrixXd B(12,4);
		Eigen::MatrixXd C(12,12);
		model_ptr->computeDynamicModel(A, B, C);		
		
		Eigen::JacobiSVD<Eigen::MatrixXd> SVD_B(B, Eigen::ComputeThinU | Eigen::ComputeThinV);
		std::cout <<"B matrix" << std::endl << B << std::endl;
		Eigen::Map<Eigen::VectorXd> x_operation_eigen(x_operation, 12, 1);


		/** Creation, mapping into an Eigen object and calculation of the linearization control signal, based on the steady state system 
			for the states at the linearization point **/

		double *u_operation;
		Eigen::Map<Eigen::MatrixXd> u_operation_eigen(u_operation, 4, 1);
		
		u_operation_eigen = SVD_B.solve(x_operation_eigen - A * x_operation_eigen);
		ROS_INFO("Fuck you code!");	
		/** Mapping of the Control Signal into an Eigen object **/
		Eigen::Map<Eigen::VectorXd> control_signal_eigen(control_signal, 4, 1);
		

		/** Creation of the absolute control signal vector, mapping into an Eigen object and calculation **/
		double *abs_control_signal;
		Eigen::Map<Eigen::VectorXd> abs_eigen_control_signal(abs_control_signal, 4, 1);
		abs_eigen_control_signal(0) = control_signal_eigen(0) + u_operation_eigen(0);
		abs_eigen_control_signal(1) = control_signal_eigen(1) + u_operation_eigen(1);
		abs_eigen_control_signal(2) = control_signal_eigen(2) + u_operation_eigen(2);
		abs_eigen_control_signal(3) = control_signal_eigen(3) + u_operation_eigen(3);
		
		
		new_state = simulator_ptr->simulatePlant(x_meas, abs_control_signal, sampling_time);
		
		x_meas[0] = new_state[0];
		x_meas[1] = new_state[1];
		x_meas[2] = new_state[2];
		x_meas[3] = new_state[3];
		x_meas[4] = new_state[4];
		x_meas[5] = new_state[5];
		x_meas[6] = new_state[6];
		x_meas[7] = new_state[7];
		x_meas[8] = new_state[8];
		x_meas[9] = new_state[9];
		x_meas[10] = new_state[10];
		x_meas[11] = new_state[11];

		//u_operation[0] = control_signal[0];
		//u_operation[1] = control_signal[1];
		//u_operation[2] = control_signal[2];
		//u_operation[3] = control_signal[3];
		
		if (mpc_pub->trylock()) {
			mpc_pub->msg_.header.stamp = ros::Time::now();
			mpc_pub->msg_.states[0] = x_meas[0];
			mpc_pub->msg_.states[1] = x_meas[1];
			mpc_pub->msg_.states[2] = x_meas[2];
			mpc_pub->msg_.states[3] = x_meas[3];
			mpc_pub->msg_.states[4] = x_meas[4];
			mpc_pub->msg_.states[5] = x_meas[5];
			mpc_pub->msg_.states[6] = x_meas[6];
			mpc_pub->msg_.states[7] = x_meas[7];
			mpc_pub->msg_.states[8] = x_meas[8];
			mpc_pub->msg_.states[9] = x_meas[9];
			mpc_pub->msg_.states[10] = x_meas[10];
			mpc_pub->msg_.states[11] = x_meas[11];

			mpc_pub->msg_.reference_states[0] = x_ref[0];
			mpc_pub->msg_.reference_states[1] = x_ref[1];
			mpc_pub->msg_.reference_states[2] = x_ref[2];
			mpc_pub->msg_.reference_states[3] = x_ref[3];
			mpc_pub->msg_.reference_states[4] = x_ref[4];
			mpc_pub->msg_.reference_states[5] = x_ref[5];
			mpc_pub->msg_.reference_states[6] = x_ref[6];
			mpc_pub->msg_.reference_states[7] = x_ref[7];
			mpc_pub->msg_.reference_states[8] = x_ref[8];
			mpc_pub->msg_.reference_states[9] = x_ref[9];
			mpc_pub->msg_.reference_states[10] = x_ref[10];
			mpc_pub->msg_.reference_states[11] = x_ref[11];

			mpc_pub->msg_.inputs[0] = control_signal[0];
			mpc_pub->msg_.inputs[1] = control_signal[1];
			mpc_pub->msg_.inputs[2] = control_signal[2];
			mpc_pub->msg_.inputs[3] = control_signal[3];
		}
		mpc_pub->unlockAndPublish();
		
		if (!ros::ok()) {			
			clock_gettime(CLOCK_REALTIME, &end_rt);
			double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
			ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);
			
			mpc_ptr->writeToDisc();
		}
	}
	
	
	return 0;
}

