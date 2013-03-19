#ifndef MPC_SIMULATOR_H
#define MPC_SIMULATOR_H

namespace mpc
{

    class Simulator
    {
	public:
	    /**
	     @brief Constructor
	     */
	     Simulator();

	    /**
	     @brief Destructor
	     */
	     ~Simulator();

	    /**
	     @brief				function used to obtain the predictions from the process model for a time horizon
	     @param int *horizonPtr		pointer to the time horizon 
	     @param MPC::Model *myModel 	pointer to the process model class "Model"
	    
	     */
	    void predictState(MPC::Model *myModel, int *horizonPtr);


	protected:
	   
	private:
	  
    };  //@class ModelPredictiveControl


}  //@namespace: mpc


#endif
