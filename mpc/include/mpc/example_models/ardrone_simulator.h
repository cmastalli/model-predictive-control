#ifndef MPC_MODEL_ARDRONESIM_H
#define MPC_MODEL_ARDRONESIM_H

#include <mpc/model/model.h>
#include <mpc/model/simulator.h>

namespace mpc
{

    namespace example_models
    {
		/**
		 This class provides methods to simulate the Parrot ARDrone1 quadrotor. 
 		 @brief This class provides methods to simulate the Parrot ARDrone1 quadrotor defined as the following non-linear system
			 \f[ \dot{x}(t) = f(x(t), u(t)) \f]
		 with initial value \f$ x(t = 0) = x_0 \f$ and given control input for the given sample \f$ u(\cdot, x_0) \f$ using a Euler backward integration method.
		 */
		class ArDroneSimulator : public mpc::model::Simulator
		{
		    public:
				/** @brief Constructor function */
				ArDroneSimulator();

				/** @brief Destructor function */
				~ArDroneSimulator() {};
				
				/**
				 @brief Function used to calculate the simulated output of the quadrotor for each time sample. In this simulator, the non linear model of the quadrotor system is implemented in this function.  
				 @param double* current_state	State vector for the system in time \f$ k \f$.
				 @param double* input_vect	Input vector for the system in time \f$ k \f$.
				 @param double sampling_time	Sampling time chosen for the simulation.
				 @return double*	Array containing the state vector for the system in time \f$ k \f$. 
				 */
				double* simulatePlant(double *current_state, double *current_input, double sampling_time);


		    protected:


		    private:
				
				/** @brief Parameter of the system called g */ 
				double g_;

				/** @brief New state vector */
				double* new_state_;

				/** @brief Number of states */
				int number_of_states_;

				/** @brief Number of inputs */
				int number_of_inputs_;

				/** @brief Inertia around the X axis */
				double Ixx_;
				
				/** @brief Inertia around the Y axis */
				double Iyy_;

				/** @brief Inertia around the Z axis */
				double Izz_;
		
				/** @brief Mass of the quadrotor */
				double m_;
			
				/** @brief Distance from the GC of the quadrotor to a rotor */
				double d_;
				
				
		};  //@class ArDroneSimulator

    }  //@namespace model

}  //@namespace mpc


#endif
