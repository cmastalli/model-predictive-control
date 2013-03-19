#ifndef MPC_SIMULATOR_H
#define MPC_SIMULATOR_H


namespace mpc
{

    /**
     @class Simulator
     @brief Abstract class to compute the dynamic model in order to make a prediction of states
     */
    class Simulator
    {
    
        public:
        
            /*
             @brief Constructor
             */
            Simulator();

            /*
             @brief Destructor
             */
	          ~Simulator();

            /*
             @brief  Function used to obtain the predictions from the process model for a time horizon
             @param int *horizon pointer to the time horizon 
             @param mpc::Model *model pointer to the process model class "Model"
             */
            void predictState(mpc::Model *model, int *horizon);


       protected:
	   
      
       private:
  
  
    };  //@class ModelPredictiveControl


}  //@namespace: mpc


#endif
