#ifndef MPC_MODELPREDICTIVECONTROL_H
#define MPC_MODELPREDICTIVECONTROL_H

#include <mpc/model/model.h>
#include <mpc/model/simulator.h>
#include <mpc/optimizer/optimizer.h>

#include <Eigen/Dense>

#include <fstream>

/**
 *  \addtogroup mpc
 *  @{
 */

//! Model Predictives Control interfaces and implementations
namespace mpc
{
    /**
     @class ModelPredictiveControl
     @brief This class serves as a base class in order to expand the functionality of the library and implement different sorts of MPC algorithms. The methods defined here are conceived in the simplest way possible to allow different implementations in the derived classes.\n
     */
    class ModelPredictiveControl
    {
        public:
            /** @brief Constructor function */
            ModelPredictiveControl() {};

            /** @brief Destructor function */
            ~ModelPredictiveControl() {};

            /**
             @brief Function to specify and set the settings of all the components within the MPC problem. The mpc::ModelPredictiveControl class can change individual parts of the MPC problem; such as the model (mpc::model::Model and derived classes), the optimizer (mpc::optimizer::Optimizer and derived classes) and, if used, the plant simulator (mpc::model::Simulator and derived classes) in order to allow different combinations of these parts when solving.
             @param mpc::model::Model *model	Pointer to the model of the plant to be used in the algorithm
             @param mpc::optimizer::Optimizer *optimizer	Pointer to the optimization library to be used in the algorithm
             @param mpc::model::Simulator *simulator	Pointer to the simulator class used to predict the states
             */
            virtual bool resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator) = 0;

            /**
             @brief Function to initialize the calculation of the MPC algorithm. The function reads all required parameters from ROS' parameter server that has been previously loaded from a configuration YAML file, and performs all the initial calculations of variables to be used in the optimization problem.
             @return Label that indicates if the MPC is initialized with success
              */
            virtual bool initMPC() = 0;

            /**
             @brief Function to update the MPC algorithm for the next iteration. The parameters defined and calculated in mpc::ModelPredictiveControl::initMPC() are used together with the methods taken from the MPC class components (mpc::model::Model, mpc::optimizer::Optimizer and mpc::model::Simulator) to find a solution to the optimization problem. This is where the different variants of MPC algorithms can be implemented in a source file from a derived class. 
			 @param double* x_measured 		State vector
			 @param double* x_reference		Reference vector
             */
            virtual void updateMPC(double* x_measured, double* x_reference) = 0;

			
			
			/**
			 @brief Function to get the control signal generated for the MPC. As the MPC algorithm states, the optimization process yields the control signals for a range of times defined by the prediction horizon, but only the current control signal is applied to the plant. This function returns the control signal for the current time.
			 @return double* Control signal for the current MPC iteration.
			 */
			virtual double* getControlSignal() const;
			
			/** @brief Function to write the data of the MPC in a text file. */
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
			
			/** @brief Data of the time vector of the system */
			std::vector<int> t_;
			
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
				outfile << "t" << '\t';
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
			outfile << t_[j] << '\t';
			
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
