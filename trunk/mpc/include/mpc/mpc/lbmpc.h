#ifndef LBMPC_QPOASES_H
#define LBMPC_QPOASES_H

#include <ros/ros.h>

#include <mpc/mpc/model_predictive_control.h>

#include <Eigen/Dense>


namespace mpc
{
    /**
     @class LBMPC
     @brief Class for solving the learning-based model predictive control problem \n
		The aim of this class is to solve the learning-based model predictive control of the following form:
		\f{eqnarray*}{
			\min_{\mathbf{u}_{k_0},\cdots,\mathbf{u}_{k_0+N-1}} J_N(x, u) & = & \frac{1}{2}(\mathbf{\tilde{x}}_{k_0+N}-\mathbf{\bar{x}}_{ref})^T\mathbf{P}(\mathbf{\tilde{x}}_{k_0+N}-\mathbf{\bar{x}}_{ref}) + \frac{1}{2}\sum\limits_{k=k_0}^{k_0+N-1} (\mathbf{\tilde{x}}_k-\mathbf{\bar{x}}_{ref})^T\mathbf{Q}(\mathbf{\tilde{x}}_k-\mathbf{\bar{x}}_{ref}) + (\mathbf{\tilde{u}}_k-\mathbf{\bar{u}}_{ref})^T\mathbf{R}(\mathbf{\tilde{u}}_k-\mathbf{\bar{u}}_{ref}) \\
			\mathbf{\tilde{x}}_{k_0} & = & \mathbf{\bar{x}}_{k_0} = \mathbf{\hat{x}}_{k_0} \\
			\mathbf{\tilde{x}}_{k+1} & = & (\mathbf{A + F)\tilde{x}}_k + (\mathbf{B + H)\check{u}}_k + \mathbf{k + z} \qquad \forall k \in [k_0, N] \\
			\mathbf{\bar{x}}_{k+1} & = & \mathbf{A\bar{x}}_k + \mathbf{B\check{u}}_k + \mathbf{k} \qquad \forall k \in [k_0, N] \\
			\mathbf{\check{u}}_k & = & \mathbf{K\bar{x}}_j + \mathbf{c}_k \qquad \forall k \in [k_0, N] \\
			\mathbf{\bar{x}_k} & \in & \mathcal{\mathbf{X}} \qquad \forall k \in [k_0, N] \\
			\mathbf{\check{u}_k} & \in & \mathcal{\mathbf{U}} \qquad \forall k \in [k_0, N] \\
			\mathbf{\bar{x}_k} & \in & \mathcal{\mathbf{X}} \ominus \mathcal{\mathbf{D}} \qquad \forall k \in [k_0, N] \\
			(\mathbf{\bar{x}_k},\xi) & \in & \omega \qquad \forall k \in [k_0, N] \\
		\f}
		To solve each of these optimal control problems the function mpc::LBMPC::initMPC initialized the control problem. The resulting optimization problem is then solved by a (predefined) minimization routine.\n
		Then the first value of the computed control is implemented and the optimization horizon is shifted forward in time. This allows the procedure to be applied iteratively and computes a (suboptimal) infinite horizon control.\n
		Note that the function mpc::LBMPC::updatedMPC can be used to computer a control signal for the next time-step. \n
     */
	class LBMPC : public mpc::ModelPredictiveControl
	{
		public:
			/**
			 @brief Constructorb function
			 @param ros::NodeHandle node	Node handle
			 */
			LBMPC(ros::NodeHandle node);
			
			/** @brief Destructor function */
			~LBMPC() {};
			
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
			
			
		private:
			/** @brief Node handle **/
			ros::NodeHandle nh_;
			
			/** @brief Label that indicates is enable the learning process **/
			bool enable_learning_process_;
			
			/** @brief Vector of pows of A-BK matrix for quadratic programming */
			std::vector<Eigen::MatrixXd> A_p_BK_pow_;
			
			/** @brief Nominal states matrix */
			Eigen::MatrixXd A_nominal_;
			
			/** @brief Estimated states matrix */
			Eigen::MatrixXd A_estimated_;
			
			/** @brief Nominal inputs matrix */
			Eigen::MatrixXd B_nominal_;
			
			/** @brief Estimated inputs matrix */
			Eigen::MatrixXd B_estimated_;
			
			/** @brief Outpus matrix */
			Eigen::MatrixXd C_nominal_;
			
			/** @brief  */
			Eigen::MatrixXd d_nominal_, d_estimated_;
			
			/** @brief Gain feedback */
			Eigen::MatrixXd K_;
			
			/** @brief Extended Q matrix for quadratic programming */
			Eigen::MatrixXd Q_bar_;
			
			/** @brief Extended R matrix for quadratic programming */
			Eigen::MatrixXd R_bar_;
			
			/** @brief Extended low constraint vector for quadratic programming */
			Eigen::VectorXd lbG_bar_;
			
			/** @brief Extended upper constraint vector for quadratic programming */
			Eigen::VectorXd ubG_bar_;
			
			/** @brief Extended low bound vector for quadratic programming */
			Eigen::VectorXd lb_bar_;
			
			/** @brief Extended upper bound vector for quadratic programming */
			Eigen::VectorXd ub_bar_;
			

			
			
	}; //@class LearningBaseMPC

} //@namespace mpc


#endif
