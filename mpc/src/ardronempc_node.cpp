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
	
	
	
	double x_ref[12] = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double x_meas[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// initial conditions
	double x_operation[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	
	// Set the initial conditions as the first linearization points
	 model_ptr->setLinearizationPoints(x_operation);

	// Having set the initial points as the linearization points, we initiate the MPC algorithm
	mpc_ptr->initMPC();
																																																																																																																																																																																																																																																				
	double *control_signal;
	double *deltacontrol_signal;
	control_signal = new double[4];
	double sampling_time = 0.0083;
	double *new_state;
	new_state = new double[12];	

	/** Creation of the delta variables for the optimization*/
	double deltaX[12];
	double deltaXref[12];

	double* x_bar;
	x_bar = new double[12];
	double* u_bar;
	u_bar = new double[4];
		
    timespec start_rt, end_rt;
    clock_gettime(CLOCK_REALTIME, &start_rt);
	for (int counter=0; counter < 1000; counter++)/*while (ros::ok())*/ {

		
 
		x_bar = model_ptr->getOperationPointsStates();
		for (int i = 0; i < 12; i++) {
			deltaX[i] = x_meas[i] - x_bar[i];
		}

		for (int i = 0; i < 12; i++) {
			deltaXref[i] = x_ref[i] - x_bar[i];
		}

		// Solving the quadratic problem to obtain the new inputs
		mpc_ptr->updateMPC(deltaX, deltaXref); // Here we are also recalculating the system matrices A and B

		
		deltacontrol_signal = mpc_ptr->getControlSignal();

		u_bar = model_ptr->getOperationPointsInputs();
		//std::cout << "operation points inputs\t" << u_bar[0] << u_bar[1] << u_bar[2] << u_bar[3] <<std::endl;

		for (int i = 0; i < 4; i++) {		
			control_signal[i] = u_bar[i] + deltacontrol_signal[i];
		}

		ROS_INFO("u=[%f,%f,%f,%f]", control_signal[0], control_signal[1], control_signal[2], control_signal[3]);
		//	Updating the simulator with the new inputs
		new_state = simulator_ptr->simulatePlant(x_meas, control_signal, sampling_time);

		// Setting the new operation points
		//model_ptr->setLinearizationPoints(new_state);
		
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
		
		/*if (!ros::ok()) {			
			
		}*/
	}

	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);
			
	mpc_ptr->writeToDisc();
	
	
	return 0;
}

