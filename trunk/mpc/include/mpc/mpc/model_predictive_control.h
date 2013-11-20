#ifndef MPC_MODELPREDICTIVECONTROL_H
#define MPC_MODELPREDICTIVECONTROL_H

#include <mpc/model/model.h>
#include <mpc/model/simulator.h>
#include <mpc/optimizer/optimizer.h>

#include <Eigen/Dense>

#include <fstream>


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
            /** @brief Constructor function */
            ModelPredictiveControl() {};

            /** @brief Constructor function */
            ~ModelPredictiveControl() {};

            /**
             @brief Function to specify the settings of all variables within the MPC problem (optimization library, horizon, etc.)
             @param mpc::model::Model *model	Pointer to the model of the plant to be used in the algorithm
             @param mpc::optimizer::Optimizer *optimizer	Pointer to the optimization library to be used in the algorithm
             @param mpc::model::Simulator *simulator	Pointer to the simulator class used to predict the states
             */
            virtual bool resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator) = 0;

            /**
             @brief Function to initialize the calculation of the MPC algorithm
             @return Label that indicates if the MPC is initialized with success
              */
            virtual bool initMPC() = 0;

            /**
             @brief Function to update the MPC algorithm for the next iteration 
			 @param double* x_measured 		State vector
			 @param double* x_reference		Reference vector
             */
            virtual void updateMPC(double* x_measured, double* x_reference) = 0;

			
			
			/**
			 @brief Function to get the control signal generates for the MPC
			 @return double* Control signal
			 */
			virtual double* getControlSignal() const;
			
			/** @brief Function to write the data of the MPC in text file */
			virtual void writeToDisc();


			
			
			
        protected:
        	/** @brief Pointer of dynamic model of the system */
			mpc::model::Model *model_;
			
			/** @brief Pointer of the simulator of the system */
			mpc::model::Simulator *simulator_;
			
			/** @brief Pointer of the optimizer of the MPC */
			mpc::optimizer::Optimizer *optimizer_;
			
			/** @brief Number of states of the dynamic model */
			int states_;
			
			/** @brief Number of inputs of the dynamic model */
			int inputs_;
			
			/** @brief Number of outputs of the dynamic model */
			int outputs_;
			
			/** @brief Horizon of prediction of the dynamic model */
			int horizon_;
			
			/** @brief Number of variables, i.e inputs * horizon */
			int variables_;
			
			/** @brief Number of constraints */
			int constraints_;

			/** @brief Vector of the operation points for the states in case of a LTI model */
			double* operation_states_;

			/** @brief Vector of the operation points for the inputs in case of a LTI model */
			double* operation_inputs_;
			
			/** @brief Infeasibility counter in the solution */
			int infeasibility_counter_;
			
			/** @brief Maximun value of the infeasibility counter */
			int infeasibility_hack_counter_max_;
			
			/** @brief States error weight matrix */
			Eigen::MatrixXd Q_;
			
			/** @brief Terminal states error weight matrix */
			Eigen::MatrixXd P_;
			
			/** @brief Input error weight matrix */
			Eigen::MatrixXd R_;
			
			/** @brief Vector of the MPC solution */
			double *mpc_solution_;
			
			/** @brief Stationary control signal for the reference state vector */
			Eigen::MatrixXd u_reference_;
			
			/** @brief Control signal computes for MPC */
			double *control_signal_;
			
			/** @brief Data of the state vector of the system */
			std::vector<std::vector<double> > x_;
			
			/** @brief Data of the reference state vector of the system */
			std::vector<std::vector<double> > xref_;
			
			/** @brief Data of the control signal vector of the system */
			std::vector<std::vector<double> > u_;
			
			/** @brief Path where the data will be save */
			std::string path_name_;
			
			/** @brief Name of the file where the data will be save */
			std::string data_name_;
			
			/** @brief Label that indicates if it will be save the data */
			bool enable_record_;
			
			
			
		private:
		

    }; //@class ModelPredictiveControl

}; //@namespace mpc


inline double* mpc::ModelPredictiveControl::getControlSignal() const
{
	for (int i = 0; i < inputs_; i++) {
		if (infeasibility_counter_ < infeasibility_hack_counter_max_)
			control_signal_[i] = mpc_solution_[infeasibility_counter_ * inputs_ + i];
		else
			control_signal_[i] = u_reference_(i);
	}

	return control_signal_;
}


inline void mpc::ModelPredictiveControl::writeToDisc()
{
	bool data = true;
	if (x_[0].size() == 0) {
		ROS_WARN("Could not save the data because the control signal information is NULL. You have to save this information at updateMPC function.");
		data = false;
	}
	if (xref_[0].size() == 0) {
		ROS_WARN("Could not save the data because the control signal information is NULL. You have to save this information at updateMPC function.");
		data = false;
	}
	if (u_[0].size() == 0) {
		ROS_WARN("Could not save the data because the control signal information is NULL. You have to save this information at updateMPC function.");
		data = false;
	}
	
	if (enable_record_ && data) {
		path_name_.append(data_name_ + std::string(".txt"));
		
		std::ofstream outfile;
		outfile.open(path_name_.c_str());
		outfile.precision(4);
		outfile.setf(std::ios::fixed, std::ios::floatfield);
		outfile.setf(std::ios::left, std::ios::adjustfield);
		for (unsigned int j = 0; j < x_[0].size(); j++) {
			if (j == 0) {
				for (unsigned int i = 0; i < x_.size(); i++)
					outfile << "x_" << i+1 << '\t';
				
				for (unsigned int i = 0; i < xref_.size(); i++)
					outfile << "xref_" << i+1 << '\t';
				
				for (unsigned int i = 0; i < u_.size(); i++) {
					if (i == u_.size() - 1)
						outfile << "u_" << i+1 << std::endl;
					else
						outfile << "u_" << i+1 << '\t';
				}
			}
	    	for (unsigned int i = 0; i < x_.size(); i++)
				outfile << x_[i][j] << '\t';
			
			for (unsigned int i = 0; i < xref_.size(); i++)
				outfile << xref_[i][j] << '\t';
			
			for (unsigned int i = 0; i < u_.size(); i++) {
				if (i == u_.size() - 1)
					outfile << u_[i][j] << std::endl;
				else
					outfile << u_[i][j] << '\t';
			}
		}
		outfile.close();
	}
}



#endif
