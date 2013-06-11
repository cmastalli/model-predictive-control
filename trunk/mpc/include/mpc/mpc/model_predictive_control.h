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
     @brief Abstract class to define the model predictive control problem
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
            void resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *optimizer, mpc::model::Simulator *simulator);

            /*
             @brief function to initialize the calculation of the MPC algorithm
             @param
             @param
             */
            virtual void initMPC();

            /*
             @brief function to update the MPC algorithm for the next iteration 
             @param
             @param
             */
            virtual void updateMPC(double* x_measured, double* x_reference);



        private:
			mpc::model::Model *model_;
			
			mpc::model::Simulator *simulator_;
			
			mpc::model::Optimizer *optimizer_;
			
			int states_, inputs_, outputs_, horizon_;
			
			Eigen::MatrixXd Q_, P_, R_;
			

    }; //@class ModelPredictiveControl

}; //@namespace mpc

