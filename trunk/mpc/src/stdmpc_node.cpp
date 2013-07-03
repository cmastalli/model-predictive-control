#include <ros/ros.h>

#include <mpc/mpc/stdmpc.h>

#include <mpc/example_models/tanks_system.h>
#include <mpc/example_models/tanks_system_simulator.h>
#include <mpc/optimizer/qpOASES.h>



/** Example for qpOASES main function using the QProblem class. */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mpc");

	ros::NodeHandle node_handle("mpc");


	mpc::model::Model *model_ptr = new mpc::example_models::TanksSystem();
	mpc::optimizer::Optimizer *optimizer_ptr = new mpc::optimizer::qpOASES(node_handle);
	mpc::model::Simulator *simulator_ptr = new mpc::example_models::TanksSystemSimulator();

	mpc::ModelPredictiveControl *mpc_ptr = new mpc::STDMPC(node_handle);

	mpc_ptr->resetMPC(model_ptr, optimizer_ptr, simulator_ptr);
	mpc_ptr->initMPC();

	double x_ref[2] = {8, 12};
	double x_meas[2] = {0, 0};


	double samplingTime = 0.02;
	double *input_k;

    timespec start_rt, end_rt;
    clock_gettime(CLOCK_REALTIME, &start_rt);
	for (int i = 0; i<5; i++){
		mpc_ptr->updateMPC(x_meas, x_ref);
		input_k = mpc_ptr->STDMPCSol_;
		x_meas = simulator_ptr->simulatePlant(x_meas, input_k, samplingTime);
	}

	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg", duration);
	
	return 0;
}

