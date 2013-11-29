#ifndef MPC_MODEL_ARDRONE_HOVERING_H
#define MPC_MODEL_ARDRONE_HOVERING_H

#include "mpc/model/model.h"
#include <Eigen/Dense>


namespace mpc
{

	namespace example_models
	{
		/**
		 @brief Class to define the example model, tanks system, of the process and the optimal control problem to be solved
		 This class gives an definition of an example model, tanks systems, of process model and the optimal control problem which shall be considered. The model itself is defined via its dynamic
		 \f{eqnarray*}{
			 \dot{x}(t) = Ax(t) + Bu(t) \\
			 y(t) = Cx(t)
   	     \f}
		 on the optimization horizon \f$ [t_0, N] \f$ with initial value \f$ x(t_0, x_0) = x_0 \f$ over an optimization criterion.
		 */
		class ArDroneHovering : public mpc::model::Model
		{
			public:
				/** @brief Constructor function */
				ArDroneHovering();
				
				
				/** @brief Destructor function */
				~ArDroneHovering();
				
				
				/**
			 	@brief After the MPC makes an iteration, this function is used to set the new linearization points for a LTV model into 				global variables for the STDMPC class 
			 	@param double* op_states 		new linearization point for the state vector
			 	*/
				virtual void setLinearizationPoints(double* op_states);

				/**
				 @brief Function to compute the dynamic model of the system
				 @param Eigen::MatrixXd& A	State matrix
				 @param Eigen::MatrixXd& B	Input matrix
				 @return bool Label that indicates if the computation of the matrices is successful
				 */
				virtual bool computeLinearSystem(Eigen::MatrixXd& A, Eigen::MatrixXd& B);


			private:

				/** @param Model coefficient */
				double c1_, c2_, c3_, c4_, c5_, c6_, c7_, c8_;
				
				/** @param Sampling time */
				double ts_;	


		}; //@class ArDroneHovering
	
	} //@namespace example_models

} //@namespace mpc


#endif



