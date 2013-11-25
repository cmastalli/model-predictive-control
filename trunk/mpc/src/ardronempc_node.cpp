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

// Global variables 
/*std::vector<std::vector<double> > controlsignal_;
std::vector<std::vector<double> > referencestates_;
std::vector<std::vector<double> > actualstates_;
std::vector<double> time_;
std::string controlsignal_data_name = "ros_workspace/model-predictive-control/mpc/data/validation/control_signal.txt";
std::string referencestates_data_name = "ros_workspace/model-predictive-control/mpc/data/reference_states.txt";
std::string actualstates_data_name = "ros_workspace/model-predictive-control/mpc/data/actual_states.txt";*/

/*bool saveToTXT()
{
	bool data = true;
	if (controlsignal_[0].size() == 0) {
		ROS_WARN("Could not save the data because the control signal information is NULL");
		data = false;
	}
	if (referencestates_[0].size() == 0) {
		ROS_WARN("Could not save the data because the reference states information is NULL");
		data = false;
	}
	if (actualstates_[0].size() == 0) {
		ROS_WARN("Could not save the data because the actual states information is NULL");
		data = false;
	}

	
	if (data) {
		std::ofstream infile;
		infile.open(controlsignal_data_name.c_str());
		infile.precision(4);
		infile.setf(std::ios::fixed, std::ios::floatfield);
		infile.setf(std::ios::left, std::ios::adjustfield);

		infile << "t\t" << "u1\t" << "u2\t" << "u3\t" << "u4\t" << std::endl;
		
		for (unsigned int j = 0; j < controlsignal_[0].size(); j++) {
			infile << time_[j];
			for (unsigned int i = 0; i < controlsignal_.size(); i++) {
				infile << '\t';
				infile << controlsignal_[i][j];
			}
			infile	<< std::endl;
		}
		infile.close();
		ROS_INFO("MPC control signal data recorded.");

		std::ofstream reffile;
		reffile.open(referencestates_data_name.c_str());
		reffile.precision(4);
		reffile.setf(std::ios::fixed, std::ios::floatfield);
		reffile.setf(std::ios::left, std::ios::adjustfield);

		reffile << "t\t" << "x\t" << "y\t" << "z\t" << "u\t" << "v\t" << "w\t" << "roll\t" << "pitch\t" << "yaw\t" << "p\t" << "q\t" << "r" << std::endl;
    	for (unsigned int j = 0; j < referencestates_[0].size(); j++) {
			reffile << time_[j];
			for (unsigned int i = 0; i < referencestates_.size(); i++) {
				reffile << '\t';
				reffile << referencestates_[i][j];
			}
			reffile	<< std::endl;
		}
		reffile.close();
		ROS_INFO("MPC reference signals data recorded.");

		std::ofstream statefile;
		statefile.open(actualstates_data_name.c_str());
		statefile.precision(4);
		statefile.setf(std::ios::fixed, std::ios::floatfield);
		statefile.setf(std::ios::left, std::ios::adjustfield);

		statefile << "t\t" << "x\t" << "y\t" << "z\t" << "u\t" << "v\t" << "w\t" << "roll\t" << "pitch\t" << "yaw\t" << "p\t" << "q\t" << "r" << std::endl;
    	for (unsigned int j = 0; j < actualstates_[0].size(); j++) {
			statefile << time_[j];
			for (unsigned int i = 0; i < actualstates_.size(); i++) {
				statefile << '\t';
				statefile << actualstates_[i][j];
			}
			statefile << std::endl;
		}
		statefile.close();
		ROS_INFO("MPC actual states data recorded.");
	}
	
	return true;
}*/

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

	/*controlsignal_.resize(4);
	referencestates_.resize(12);
	actualstates_.resize(12);
	time_.push_back(0.0);
	for (int j = 0; j < 4; j++) {
		controlsignal_[j].push_back(0.0);
	}
	for (int j = 0; j < 12; j++) {
		referencestates_[j].push_back(0.0);
		actualstates_[j].push_back(0.0);
 	}*/
	
	double x_ref[12] = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double x_meas[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// initial conditions
	double x_operation[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	
	// Set the initial conditions as the first linearization points
	 model_ptr->setLinearizationPoints(x_ref);

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
		mpc_ptr->updateMPC(x_meas, x_ref); // Here we are also recalculating the system matrices A and B

		
		control_signal = mpc_ptr->getControlSignal();

		u_bar = model_ptr->getOperationPointsInputs();
		//std::cout << "operation points inputs\t" << u_bar[0] << u_bar[1] << u_bar[2] << u_bar[3] <<std::endl;

		/*for (int i = 0; i < 4; i++) {		
			control_signal[i] = u_bar[i] + deltacontrol_signal[i];
		}*/

		//ROS_INFO("u=[%f,%f,%f,%f]", u_bar[0], u_bar[1], u_bar[2], u_bar[3]);
		//ROS_INFO("u=[%f,%f,%f,%f]", deltacontrol_signal[0], deltacontrol_signal[1], deltacontrol_signal[2], deltacontrol_signal[3]);
		//	Updating the simulator with the new inputs
		new_state = simulator_ptr->simulatePlant(x_meas, control_signal, sampling_time);

		Eigen::Map<Eigen::VectorXd> new_(new_state, 12);
		std::cout << "New state\t" << new_.transpose() << std::endl;
		// Setting the new operation points
		//model_ptr->setLinearizationPoints(new_state);
		

		if (mpc_pub->trylock()) {
			mpc_pub->msg_.header.stamp = ros::Time::now();

			for (int j = 0; j < 12; j++) {
				mpc_pub->msg_.states[j] = x_meas[j];
			}

			for (int j = 0; j < 12; j++) {
				mpc_pub->msg_.reference_states[j] = x_ref[j];
			}

			for (int j = 0; j < 4; j++) {
				ROS_INFO("u(%d)=[%f]", j, control_signal[j]);
				mpc_pub->msg_.inputs[j] = control_signal[j];
			}
			
		}
		mpc_pub->unlockAndPublish();

		/*for (int j = 0; j < 12; j++) {
				referencestates_[j][counter] = x_ref[j];
				actualstates_[j][counter] = x_meas[j];
			}

		for (int j = 0; j < 4; j++) {
				controlsignal_[j][counter] = control_signal[j];
		}*/

		// Shifting the state vector 
		for (int i = 0; i < 12; i++) {
				x_meas[i] = new_state[i];
		}
		
		/*if (!ros::ok()) {			
			
		}*/
	}

	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);
			
	mpc_ptr->writeToDisc();
	/*if (saveToTXT()){
		ROS_INFO("Successful recording.");
	}*/
	
	
	return 0;
}



