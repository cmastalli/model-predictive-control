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
			STDMPC();
			
			// Destructor
			~STDMPC() {
				delete lbA_;
				delete ubA_;

				delete lb_;
				delete ub_;

				delete lbA_bar_;
				delete ubA_bar_;

				delete lb_bar_;
				delete ub_bar_;
				
				delete G_bar_;
			}
			
			virtual void resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator);
			
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
			int nConst_;

			/** Number of variables (= horizon_*inputs_) **/
			int nVar_;

			/** Prediction horizon for the algorithm **/
			int horizon_;

		private:
			ros::NodeHandle nh_;
			
			mpc::model::Model *model_;

			mpc::optimizer::Optimizer *optimizer_;

			mpc::model::Simulator *simulator_;
			
			int states_, inputs_, outputs_;

			/** Constraint vectors**/
			double *lbA_, *ubA_;

			/** Bound vectors **/
			double *lb_, *ub_;

			/** Extended constraint vectors **/
			double *lbA_bar_, *ubA_bar_;

			/** Extended bound vectors **/
			double *lb_bar_, *ub_bar_;

			/** Extended constraint matrix **/
			double *G_bar_; 

			/** Extended A matrix **/
			Eigen::MatrixXd A_bar_;

			/** Extended B matrix **/
			Eigen::MatrixXd B_bar_;

			/** Extended Q matrix **/
			Eigen::MatrixXd Q_bar_;

			/** Hessian Matrix array **/
			double *H_bar_;

			/** Gradient Matrix array **/
			double *g_;

			
			
	}; //@class StandardMPC

} //@namespace mpc

#endif

