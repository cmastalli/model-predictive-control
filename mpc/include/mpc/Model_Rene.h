#ifndef MPC_MODEL_H
#define MPC_MODEL_H

namespace mpc
{
    /**
     @class Model
     @brief Abstract class to define the model of the process and the optimal control problem to be solved
     */
    class Model
    {
        public:
            /**
             @brief Constructor function
             */
            Model();

            /**
             @brief Destructor function
             */
            ~Model();
	
	          /*
             @brief function to define the cost function associated to the MPC problem 
             @param H
             @param z
             @param g
             */
            virtual void setCostFunction();

            /*
             @brief function to set the whole optimization problem according to the documentation presented by qpOASES
             @param
             @param
             */
            virtual void setOptProblem();
 
            /*
             @brief function that defines the behavior of the process model
             @param 
             @param
             */
            virtual void setDynamicFunction();
        

        protected:


        private:


    }; // Model Predictive Control class

}; // mpc namespace
