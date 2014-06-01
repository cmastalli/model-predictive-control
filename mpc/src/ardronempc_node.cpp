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
#include <qpOASES/Utils.hpp>

USING_NAMESPACE_QPOASES



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
	
	// Operation state
	double x_operation[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		
	// Set the initial conditions as the first linearization points in order to initiate the MPC algorithm
	model_ptr->setLinearizationPoints(x_operation);
	mpc_ptr->initMPC();
	
	double x_ref[12] = {0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double x_meas[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	double *u, *delta_u, *x_bar, *u_bar, *new_state;
	double delta_xmeas[12], delta_xref[12];
	double sampling_time = 0.0083;
	
	u = new double[4];
	new_state = new double[12];	
	x_bar = new double[12];
	u_bar = new double[4];

	double t_sim;
	
			
    timespec start_rt, end_rt;
    clock_gettime(CLOCK_REALTIME, &start_rt);
	real_t begin, end;
    
	for (int counter = 0; counter < 4000; counter++) {
		/*while (ros::ok())*/

		
		t_sim = sampling_time * counter;

		if (t_sim < 8){
			x_ref[0] = 0.25*t_sim;			// referencia de posicion en X
			x_ref[1] = 0.;			// referencia de posicion en Y
			x_ref[3] = 0.25;	// referencia de velocidad en X (rampa ascendente)
			x_ref[4] = 0.;
		}
		else if (t_sim >= 8 && t_sim < 16){
			x_ref[0] = 2.0;			// referencia de posicion en X
			x_ref[1] = -0.25*t_sim + 2.0;			// referencia de posicion en Y
			x_ref[3] = 0.;	// referencia de velocidad en X 
			x_ref[4] = -0.25;
		}
		else if (t_sim >= 16 && t_sim < 24){
			x_ref[0] = -0.25*t_sim + 6.0;			// referencia de posicion en X
			x_ref[1] = -2.0;			// referencia de posicion en Y
			x_ref[3] = -0.25;	// referencia de velocidad en X 
			x_ref[4] = 0.;
		}
		else if (t_sim >= 24 && t_sim < 32){
			x_ref[0] = 0.;			// referencia de posicion en X
			x_ref[1] = 0.25*t_sim - 8.0;			// referencia de posicion en Y
			x_ref[3] = 0.;	// referencia de velocidad en X 
			x_ref[4] = 0.25;
		}
		/////////////////////////////////////////////////////////////////////////////////
		
		else {
			x_ref[0] = 0.;			
			x_ref[1] = 0.;			
			x_ref[3] = 0.;
			x_ref[4] = 0.;
		}


		
		x_bar = model_ptr->getOperationPointsStates(); 
		for (int i = 0; i < 12; i++) {
			delta_xmeas[i] = x_meas[i] - x_bar[i];
			delta_xref[i] = x_ref[i] - x_bar[i];
		}

		// Solving the quadratic problem to obtain the new inputs
		begin = getCPUtime();
		mpc_ptr->updateMPC(delta_xmeas, delta_xref); // Here we are also recalculating the system matrices A and B
		end = getCPUtime();
		real_t duration = end - begin;
		//std::cout << "Optimization problem computational time:" << static_cast<double>(duration) << std::endl;

		delta_u = mpc_ptr->getControlSignal();
		u_bar = model_ptr->getOperationPointsInputs();
		for (int i = 0; i < 4; i++)
			u[i] = u_bar[i] + delta_u[i];
		
		//	Updating the simulator with the new inputs
		new_state = simulator_ptr->simulatePlant(x_meas, u, sampling_time);

		Eigen::Map<Eigen::VectorXd> new_(new_state, 12);
		//std::cout << "New state\t" << new_.transpose() << std::endl;
		
		// Setting the new operation points
//		model_ptr->setLinearizationPoints(new_state);

		if (mpc_pub->trylock()) {
			mpc_pub->msg_.header.stamp = ros::Time::now();
			for (int j = 0; j < 12; j++) {
				mpc_pub->msg_.states[j] = x_meas[j];
				mpc_pub->msg_.reference_states[j] = x_ref[j];
			}				
			for (int j = 0; j < 4; j++)
				mpc_pub->msg_.inputs[j] = u[j];
			
		}
		mpc_pub->unlockAndPublish();
		
		
		// Shifting the state vector 
		for (int i = 0; i < 12; i++) {
			x_meas[i] = new_state[i];
		}
	}

	//clock_gettime(CLOCK_REALTIME, &end_rt);
	//double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);
			
	mpc_ptr->writeToDisc();
		
	
	return 0;
}
