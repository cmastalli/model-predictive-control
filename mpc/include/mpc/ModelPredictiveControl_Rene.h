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
             @param MPC::Model *myModel		pointer to the model of the plant to be used in the algorithm
             @param MPC::Optimizer *qpOASES		pointer to the optimization library to be used in the algorithm
             @param MPC::Simulator *mySimulator	pointer to the simulator class used to predict the states
             */
            void resetMPC( MPC::Model *myModel, MPC::Optimizer *qpOASES, MPC::Simulator *mySimulator);

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


    }; // Model Predictive Control class

}; // mpc namespace
