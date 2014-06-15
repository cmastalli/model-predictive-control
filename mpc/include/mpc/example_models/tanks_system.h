#ifndef TANKSSYSTEM_H
#define TANKSSYSTEM_H

#include "mpc/model/model.h"


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
		 @brief Class to define the process model of the tank system available at Simon Bolivar University's Automatic Control Lab.
		 */
		class TanksSystem : public mpc::model::Model
		{
			public:
				/** @brief Constructor function */
				TanksSystem();
				
				
				/** @brief Destructor function */
				~TanksSystem();


				/**
			 	@brief After the MPC makes an iteration, this function is used to set the new linearization points for a LTV model into 				global variables. 
			 	@param double* op_states 		new linearization point for the state vector
			 	*/
				virtual void setLinearizationPoints(double* op_states);
				
				/**
				 @brief Function to compute the dynamic model of the system.
				 @param Eigen::MatrixXd& A	State matrix
				 @param Eigen::MatrixXd& B	Input matrix
				 @return bool Label that indicates if the computation of the matrices is successful
				 */
				virtual bool computeLinearSystem(Eigen::MatrixXd& A, Eigen::MatrixXd& B);


			private:


		}; //@class TanksSystem
	
	} //@namespace example_models

} //@namespace mpc
/** @} End of Doxygen Groups*/

#endif



