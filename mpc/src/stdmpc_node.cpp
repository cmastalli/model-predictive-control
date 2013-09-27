#include <ros/ros.h>

#include <mpc/mpc/stdmpc.h>

#include <mpc/example_models/tanks_system.h>
#include <mpc/example_models/tanks_system_simulator.h>
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
	mpc_pub->msg_.states.resize(2);
	mpc_pub->msg_.reference_states.resize(2);
	mpc_pub->msg_.inputs.resize(1);
	mpc_pub->unlock();
	
	
	mpc::model::Model *model_ptr = new mpc::example_models::TanksSystem();
	mpc::optimizer::Optimizer *optimizer_ptr = new mpc::optimizer::qpOASES(node_handle);
	mpc::model::Simulator *simulator_ptr = new mpc::example_models::TanksSystemSimulator();
	
	mpc::ModelPredictiveControl *mpc_ptr = new mpc::STDMPC(node_handle);
	
	
	mpc_ptr->resetMPC(model_ptr, optimizer_ptr, simulator_ptr);
	mpc_ptr->initMPC();
	
	
	double x_ref[12] = {4.0, 4.0};
	double x_meas[12] = {0., 0.};
	
	double sampling_time = 0.01;
	double *control_signal;
	
	double *new_state;
	
		
    timespec start_rt, end_rt;
    clock_gettime(CLOCK_REALTIME, &start_rt);
	while (ros::ok()) {
		mpc_ptr->updateMPC(x_meas, x_ref);
		control_signal = mpc_ptr->getControlSignal();
		new_state = simulator_ptr->simulatePlant(x_meas, control_signal, sampling_time);
		
		x_meas[0] = new_state[0];
		x_meas[1] = new_state[1];
	
		
		if (mpc_pub->trylock()) {
			mpc_pub->msg_.header.stamp = ros::Time::now();
			mpc_pub->msg_.states[0] = x_meas[0];
			mpc_pub->msg_.states[1] = x_meas[1];

			mpc_pub->msg_.reference_states[0] = x_ref[0];
			mpc_pub->msg_.reference_states[1] = x_ref[1];

			mpc_pub->msg_.inputs[0] = control_signal[0];
			
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

