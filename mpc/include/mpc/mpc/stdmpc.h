#ifndef STDMPC_QPOASES_H
#define STDMPC_QPOASES_H

#include <ros/ros.h>

#include <mpc/mpc/model_predictive_control.h>

#include <Eigen/Dense>
#include <cmath>



namespace mpc
{
    /**
     @class STDMPC
     @brief Class for solving the explicit model predictive control problem \n
		The aim of this class is to solve the explicit model predictive control of the following form:
		\f{eqnarray*}{
			\min_{\mathbf{u}_{k_0},\cdots,\mathbf{u}_{k_0+N-1}} J_N(x, u) & = & \frac{1}{2}(\mathbf{x}_{k_0+N}-\mathbf{x}_{ref})^T\mathbf{P}(\mathbf{x}_{k_0+N}-\mathbf{x}_{ref}) + \frac{1}{2}\sum\limits_{k=k_0}^{k_0+N-1} (\mathbf{x}_k-\mathbf{x}_{ref})^T\mathbf{Q}(\mathbf{x}_k-\mathbf{x}_{ref}) + (\mathbf{u}_k-\mathbf{u}_{ref})^T\mathbf{R}(\mathbf{u}_k-\mathbf{u}_{ref}) \\
			\mathbf{x}_{k_0} & = & \omega_0(k_0) \\
			\mathbf{x}_{k+1} & = & \mathbf{Ax}_k + \mathbf{Bu}_k \qquad \forall k \in [k_0, N] \\
			\bar{x} & \leq & \mathbf{Mx}_k \qquad \forall k \in [k_0, N] \\
			\bar{u} & \leq & \mathbf{Nu}_k \qquad \forall k \in [k_0, N] \\
		\f}
		To solve each of these optimal control problems the function mpc::STDMPC::initMPC initialized the control problem. The resulting optimization problem is then solved by a (predefined) minimization routine.\n
		Then the first value of the computed control is implemented and the optimization horizon is shifted forward in time. This allows the procedure to be applied iteratively and computes a (suboptimal) infinite horizon control.\n
		Note that the function mpc::STDMPC::updatedMPC can be used to computer a control signal for the next time-step. \n
     */
	class STDMPC : public mpc::ModelPredictiveControl
	{

		public:
			/** @brief Constructor function */
			STDMPC(ros::NodeHandle node_handle);
			
			/** @brief Destructor function */
			~STDMPC();
			
			/*
             @brief Function to specify the settings of all variables within the MPC problem (optimization library, horizon, etc.)
             @param mpc::model::Model *model pointer to the model of the plant to be used in the algorithm
             @param mpc::optimizer::Optimizer *optimizer pointer to the optimization library to be used in the algorithm
             @param mpc::model::Simulator *simulator pointer to the simulator class used to predict the states
             */
			virtual bool resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator);
			
			/* @brief function to initialize the calculation of the MPC algorithm */
			virtual bool initMPC();			
			
			/**
			 @brief Function to solve the optimization problem formulated in the MPC  
			 @param double* x_measured 		state vector
			 @param double* x_reference		reference vector
			 */
			virtual void updateMPC(double* x_measured, double* x_reference);
						
			
			
		protected:


			
			
			
		private:
			/** @brief Node handle **/
			ros::NodeHandle nh_;
			
			/** @brief States matrix */
			Eigen::MatrixXd A_;
			
			/** @brief Inputs matrix */
			Eigen::MatrixXd B_;
			
			/** @brief Outputs matrix */
			Eigen::MatrixXd C_;

			/** @brief Extended A matrix for quadratic programming */
			Eigen::MatrixXd A_bar_;
			
			/** @brief Extended B matrix for quadratic programming */
			Eigen::MatrixXd B_bar_;

			/** @brief Extended Q matrix for quadratic programming */
			Eigen::MatrixXd Q_bar_;
			
			/** @brief Extended A matrix for quadratic programming */
			Eigen::MatrixXd R_bar_;
			
			/** @brief Extended M matrix for quadratic programming */
			Eigen::MatrixXd M_bar_;
			
			/** @brief Extended low constraint vector for quadratic programming */
			Eigen::VectorXd lbG_bar_;
			
			/** @brief Extended upper constraint vector for quadratic programming */
			Eigen::VectorXd ubG_bar_;
			
			/** @brief Extended low bound vector for quadratic programming */
			Eigen::VectorXd lb_bar_;
			
			/** @brief Extended upper bound vector for quadratic programming */
			Eigen::VectorXd ub_bar_;

			/** @brief Vector of pows of state matrix for quadratic programming */
			std::vector<Eigen::MatrixXd> A_pow_;
						
			
	}; //@class STDMPC

} //@namespace mpc

#endif

