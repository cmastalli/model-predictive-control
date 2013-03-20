#ifndef MPC_MODELPREDICTIVECONTROL_H
#define MPC_MODELPREDICTIVECONTROL_H

namespace mpc
{
    /**
     @class ModelPredictiveControl
     @brief Abstract (I'm not sure!!) class to define the model predictive control problem
     */
    class ModelPredictiveControl
    {
        public:
        
            /**
             @brief Constructor function
             */
            ModelPredictiveControl();

            /**
             @brief Constructor function
             */
            ~ModelPredictiveControl();

            /*
             @brief Function to specify the settings of all variables within the MPC problem (optimization library, horizon, etc.)
             @param mpc::model::Model *model pointer to the model of the plant to be used in the algorithm
             @param mpc::optimizer::Optimizer *qpOASES pointer to the optimization library to be used in the algorithm
             @param mpc::model::Simulator *simulator pointer to the simulator class used to predict the states
             */
            void resetMPC(mpc::model::Model *model, mpc::optimizer::Optimizer *qpOASES, mpc::model::Simulator *simulator);

            /*
             @brief function to initialize the calculation of the MPC algorithm
             @param
             @param
             */
            void initMPCCalc();

            /*
             @brief function to update the MPC algorithm for the next iteration 
             @param
             @param
             */
            void updateMPC();


        protected:


        private:


    }; //@class ModelPredictiveControl

}; //@namespace mpc

