#include <ros/ros.h>

#include <mpc/mpc/model_predictive_control.h>
#include <mpc/mpc/lbmpc.h>

#include <mpc/example_models/tanks_system.h>
#include <mpc/example_models/tanks_system_simulator.h>
#include <mpc/optimizer/qpOASES.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "lbmpc");

	ros::NodeHandle node_handle("mpc");
	
	
	mpc::model::Model *model = new mpc::example_models::TanksSystem(node_handle);
	mpc::model::Simulator *simulator = new mpc::example_models::TanksSystemSimulator(model);
	mpc::optimizer::Optimizer *optimizer = new mpc::optimizer::qpOASES(node_handle, model);
	
	
	mpc::ModelPredictiveControl *mpc = new mpc::LBMPC(node_handle);

	mpc->resetMPC(model, optimizer, simulator);
	mpc->initMPC();
	
	double x_ref[2] = {7, 7};
	double x_meas[2] = {8, 12};
	
    timespec start_rt, end_rt;
    clock_gettime(CLOCK_REALTIME, &start_rt);
    
	mpc->updateMPC(x_meas, x_ref);
	
	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg", duration);
	
	return 0;
}
