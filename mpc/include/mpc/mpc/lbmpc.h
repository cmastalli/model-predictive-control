#ifndef LBMPC_QPOASES_H
#define LBMPC_QPOASES_H

#include <ros/ros.h>

#include <mpc/mpc/model_predictive_control.h>

#include <Eigen/Dense>


namespace mpc
{

	class LBMPC : protected mpc::ModelPredictiveControl
	{

		public:
			LBMPC(ros::NodeHandle node);
			
			~LBMPC() {};
			
			virtual void resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator);
			
			virtual bool initMPC();			
			
			/**
			 @brief Function to solve the optimization problem formulated in the MPC  
			 @param Eigen::VectorXd x_measured 		state vector
			 @param Eigen::VectorXd x_reference		reference vector 
			 */
			virtual void updateMPC(double* x_measured, double* x_reference);
			
			
		private:
			ros::NodeHandle nh_;
			
			bool enable_learning_process_;
			
			std::vector<Eigen::MatrixXd> A_p_BK_pow_;
			
			Eigen::MatrixXd A_estimated_, A_nominal_;
			
			Eigen::MatrixXd B_estimated_, B_nominal_;
			
			Eigen::MatrixXd d_nominal_, d_estimated_;
			
			Eigen::MatrixXd K_;
			
			
	}; //@class LearningBaseMPC

} //@namespace mpc


#endif
