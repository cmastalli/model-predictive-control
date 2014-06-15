#ifndef MPC_MODEL_ARDRONE_H
#define MPC_MODEL_ARDRONE_H

#include "mpc/model/model.h"
#include <Eigen/Dense>

/**
 *  \addtogroup mpc
 *  @{
 */

//! Model Predictives Control interfaces and implementations
namespace mpc
{
	/**
	 *  \addtogroup example_models
	 *  @{
	 */
	 //* Example models implementations
	namespace example_models
	{
		/**
		 @brief Derived class from mpc::model::Model that represents the dynamics of Parrot's ARDrone1 quadrotor.
		 */
		class ArDrone : public mpc::model::Model
		{
			public:
				/** @brief Constructor function */
				ArDrone();
				
				
				/** @brief Destructor function */
				~ArDrone();
				
				
				/**
			 	@brief After the MPC completes an iteration, this function is used to set the new linearization points for a LTV model as global variables. 
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

				/** @param Thrust coefficient */
				double Ct_;	

				/** @param Drag coefficient */
				double Cq_;

				/** @param Inertia of the quadrotor around the X axis */
				double Ixx_;

				/** @param Inertia of the quadrotor around the Y axis */
				double Iyy_;

				/** @param Inertia of the quadrotor around the Z axis */
				double Izz_;

				/** @param Mass of the quadrotor */
				double m_;

				/** @param Distance from the GC to the rotors */
				double d_;

				/** @param gravity */
				double g_;

				/** @param Sampling time */
				double ts_;

		}; //@class ArDrone
	
	} //@namespace example_models

} //@namespace mpc
/** @} End of Doxygen Groups*/

#endif



