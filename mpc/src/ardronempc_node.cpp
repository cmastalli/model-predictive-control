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
	
	
	double x_ref[12] = {0., 0., 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.};
	double x_meas[12] = {0., 0.,  0.0, 0., 0., 0., 0., 0., 0., 0., 0., 0.};

	// initial conditions
	double x_operation[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double u_operation[4] = {0.0, 0.0, 0.0, 0.0};

	// Set the initial conditions as the first linearization points
	 mpc_ptr->setLinearizationPoints(x_operation);
																																																																																																																																																																																																																																																				
	double *control_signal;
	control_signal = new double[4];
	double sampling_time = 0.0;
	double *new_state;
	new_state = new double[12];	
		
    timespec start_rt, end_rt;
    clock_gettime(CLOCK_REALTIME, &start_rt);
	while (ros::ok()) {
		//model_ptr->setStates(x_meas);
		//model_ptr->setInputs(u_operation); 

		/*double x_lin_meas[12];
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
		x_lin_ref[11] = x_ref[11] - x_operation[11];*/
		
		// Solving the quadratic problem to obtain the new inputs
		mpc_ptr->updateMPC(x_meas, x_ref); // Here we are also recalculating the system matrices A and B

		ROS_INFO("u=[%f,%f,%f,%f]", control_signal[0], control_signal[1], control_signal[2], control_signal[3]);
		control_signal = mpc_ptr->getControlSignal();
		
		//	Updating the simulator with the new inputs
		new_state = simulator_ptr->simulatePlant(x_meas, control_signal, sampling_time);

		// Setting the new operation points
		mpc_ptr->setLinearizationPoints(new_state);
		
		// Shifting the state vector 
		for (int i = 0; i < 12; i++) {
		x_meas[i] = new_state[i];
		}
		

		if (mpc_pub->trylock()) {
			mpc_pub->msg_.header.stamp = ros::Time::now();

			for (int j = 0; j < 12; j++) {
				mpc_pub->msg_.states[j] = x_meas[j];
			}

			for (int j = 0; j < 12; j++) {
				mpc_pub->msg_.reference_states[j] = x_ref[j];
			}

			for (int j = 0; j < 4; j++) {
				mpc_pub->msg_.inputs[j] = control_signal[j];
			}
			
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

