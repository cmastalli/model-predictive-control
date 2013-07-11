#ifndef MPC_MODEL_SIMULATOR_H
#define MPC_MODEL_SIMULATOR_H

#include <mpc/model/model.h>

namespace mpc
{

    namespace model
    {
		/**
		 @brief This class provides methods to simulate a given model of a process defined by a class mpc::model::Model object
		 This class provides methods to simulate a given model
		 \f[ \dot{x}(t) = Ax(t) + Bu(t) \f]
		 \f[ y(t) = Cx(t) \f]
		 on a fixed prediction horizon interval \f$ [t_0, t_N] \f$ with initial value \f$ x(t_0, x_0) = x_0 \f$ and given control \f$ u(\cdot, x_0) \f$. That is, for a given class mpc::model::Model object and a given control \f$ u \f$ the simulator can solve the differential or difference equation forward in time.
		 */
		class Simulator
		{
		    public:
				/** @brief Constructor function */
				Simulator() {};

				/** @brief Destructor function */
				~Simulator() {};

				/**
				 @brief Function used to simulate the specified plant 
				 @param double* state_vect	State vector
				 @param double* input_vect	Input vector
				 @param double sampling_time	Sampling time
				 @return double*	New state vector
				 */
				virtual double* simulatePlant(double *state_vect, double *input_vect, double sampling_time) = 0;


		    protected:
		    	/** @brief New state vector */
				double *new_state_;
			
			
		    private:
			
			
		};  //@class ModelPredictiveControl

    }  //@namespace model

}  //@namespace mpc


#endif
