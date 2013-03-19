#ifndef MPC_SIMULATOR_H
#define MPC_SIMULATOR_H

namespace mpc
{

 	8
 	9
 	10
 	11
 	12
 	13
 	14
 	15
 	16
 	17
 	18
 	19
 	20
 	21
 	22
 	23
 	24
 	25
 	26
 	27
 	28
 	29
 	30
 	31
 	32
 	33
 	34
 	35
 	36
 	37
 	38
 	39
 	40
 	41
 	42
 	43

	

#ifndef MPC_OPTIMIZER_H
#define MPC_OPTIMIZER_H

namespace mpc
{

    /**
     @class Simulator
     @brief Abstract class to compute the dynamic model in order to make a prediction of states
     */
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
