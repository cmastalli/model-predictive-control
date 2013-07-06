#ifndef STDMPC_QPOASES_H
#define STDMPC_QPOASES_H

#include <ros/ros.h>

#include <mpc/mpc/model_predictive_control.h>

#include <Eigen/Dense>


namespace mpc
{

	class STDMPC : public mpc::ModelPredictiveControl
	{

		public:
			// Constructor
			STDMPC(ros::NodeHandle node_handle);
			
			// Destructor
			~STDMPC();
			
			virtual bool resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator);
			
			virtual bool initMPC();			
			
			/**
			 @brief Function to solve the optimization problem formulated in the MPC  
			 @param Eigen::VectorXd x_measured 		state vector
			 @param Eigen::VectorXd x_reference		reference vector 
			 */
			virtual void updateMPC(double* x_measured, double* x_reference);
			//virtual void updateMPC(Eigen::MatrixXd x_measured, Eigen::MatrixXd x_reference);
			
			
		protected:
			/** Number of constraints **/
			int constraints_;

			/** Number of variables (= horizon_*inputs_) **/
			int variables_;

			/** Prediction horizon for the algorithm **/
			int horizon_;
			
			
		private:
			/** Node handle **/
			ros::NodeHandle nh_;
			
			Eigen::MatrixXd A_, B_, C_;

			/** Extended A and B matrices **/
			Eigen::MatrixXd A_bar_, B_bar_;

			/** Extended Q and R matrix **/
			Eigen::MatrixXd Q_bar_, R_bar_;
			

			Eigen::MatrixXd M_bar_;
			

			Eigen::VectorXd lbG_bar_, ubG_bar_;
			

			Eigen::VectorXd lb_bar_, ub_bar_;

			
			std::vector<Eigen::MatrixXd> A_pow_;
			
	}; //@class StandardMPC

} //@namespace mpc

#endif

