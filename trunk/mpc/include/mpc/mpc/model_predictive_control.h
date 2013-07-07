#ifndef MPC_MODELPREDICTIVECONTROL_H
#define MPC_MODELPREDICTIVECONTROL_H

#include <mpc/model/model.h>
#include <mpc/model/simulator.h>
#include <mpc/optimizer/optimizer.h>

#include <Eigen/Dense>


namespace mpc
{
    /**
     @class ModelPredictiveControl
     @brief Abstract class for solving the model predictive control problem \n
		The aim of this class is to solve the following optimal control problem with infinite optimization horizon for a given initial value \f$ x(t_0) = x_0 \f$ for fast-dynamic systems such as in robotics applications
		\f{eqnarray*}{
			\mbox{Minimize} \; J_\infty(x, u) & = & \sum\limits_{i=0}^{\infty} \int\limits_{t_i}^{t_{i + 1}} L \left( \tau, x_{u}(\tau, x_0), u(\tau, x_0) \right) d \tau + \sum\limits_{i=0}^{\infty} l \left( t_k, x_{u}(t_i, x_0), u(t_i, x_0) \right) \\
			\dot{x}_{u}(t) & = & f(x_{u}(t, x(t_k)), u(x(t_k), t)) \qquad \forall t \in [t_0, \infty) \\
			x_{u}(0, x_0) & = & x_0 \\
			x_{u}(t, x_0) & \in & X \qquad \forall t \in [t_0, \infty] \\
			u(t, x_0) & \in & U \qquad \forall t \in [t_0, \infty)
		\f}
		Since solving this problem usually requires the solution of a Hamilton-Jacobi-Bellman (partial) differential equation, we use a model predictive control approach an approximate the infinite horizon solution by the solution of a sequence of finite horizon optimal control problems:
		\f{eqnarray*}{
			\mbox{Find} \; \mu(x(t_k)) & := & u_{[0]} \\
			\mbox{ST.} \;\; u_{[0, N-1]} & = & \mbox{{\it argmin}}_{u \in \mathcal{U}_N} J_N (x(t_k), u) \\
			J_N (x(t_k), u) & = & \sum\limits_{i=0}^{N - 1} \int\limits_{t_i^k}^{t_{i + 1}^k} L \left( \tau, x_{u}(\tau, x(t_k)), u(\tau, x(t_k)) \right) d \tau \\
			&& + \sum\limits_{i=0}^{N - 1} l \left( t_i^k, x_{u}(t_i^k, x(t_k)), u(t_i^k, x(t_k)) \right) + F(t_N^k, x_{u}(t_N^k, x(t_k))) \\
			\dot{x}_{u}(t) & = & f(t, x_{u}(t, x(t_k)), u(x(t_k), t)) \qquad \forall t \in [t_0^k, t_N^k] \\
			x_{u}(0, x(t_k)) & = & x(t_k) \\
			x_{u}(t, x(t_k)) & \in & X \qquad \forall t \in [t_0^k, t_N^k] \\
			u(x(t_k), t) & \in & U \qquad \forall t \in [t_i^k, t_{i + 1}^k)
			\f}
		To solve each of these optimal control problems the function mpc::ModelPredictiveControl::initMPC initialized the control problem. The resulting optimization problem is then solved by a (predefined) minimization routine.\n
		Then the first value of the computed control is implemented and the optimization horizon is shifted forward in time. This allows the procedure to be applied iteratively and computes a (suboptimal) infinite horizon control.\n
		Note that the function mpc::ModelPredictiveControl::updatedMPC can be used to computer a control signal for the next time-step. \n
     */
    class ModelPredictiveControl
    {
        public:
        
            /**
             @brief Constructor function
             */
            ModelPredictiveControl() {};

            /**
             @brief Constructor function
             */
            ~ModelPredictiveControl() {};

            /*
             @brief Function to specify the settings of all variables within the MPC problem (optimization library, horizon, etc.)
             @param mpc::model::Model *model pointer to the model of the plant to be used in the algorithm
             @param mpc::optimizer::Optimizer *optimizer pointer to the optimization library to be used in the algorithm
             @param mpc::model::Simulator *simulator pointer to the simulator class used to predict the states
             */
            virtual bool resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator) = 0;

            /* @brief function to initialize the calculation of the MPC algorithm */
            virtual bool initMPC() = 0;

            /*
             @brief function to update the MPC algorithm for the next iteration 
			 @param double* x_measured 		state vector
			 @param double* x_reference		reference vector
             */
            virtual void updateMPC(double* x_measured, double* x_reference) = 0;
			
			
			virtual double* getControlSignal() const;


        protected:
			mpc::model::Model *model_;
			
			mpc::model::Simulator *simulator_;
			
			mpc::optimizer::Optimizer *optimizer_;
			
			int states_, inputs_, outputs_, horizon_;
			
			Eigen::MatrixXd Q_, P_, R_;

			double* mpc_solution_;
						
			
		private:
		

    }; //@class ModelPredictiveControl

}; //@namespace mpc


inline double* mpc::ModelPredictiveControl::getControlSignal() const
{
	return mpc_solution_;
}


#endif
