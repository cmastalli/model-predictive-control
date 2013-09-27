#ifndef MPC_MODEL_ARDRONE_H
#define MPC_MODEL_ARDRONE_H

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
		class ArDrone : public mpc::model::Model
		{
			public:
				/** @brief Constructor function */
				ArDrone();
				
				
				/** @brief Destructor function */
				~ArDrone();
				
				/** 
				 @brief Function to compute at each instant the linear model of the quadrotor.
				 @param double* states_measured	The current state of the quadrotor.
                **/
				void computeLTIModel();		


				/**
				 @brief Functio to compute the dynamic model of the system
				 @param Eigen::MatrixXd& A	State matrix
				 @param Eigen::MatrixXd& B	Input matrix
				 @param Eigen::MatrixXd& C	Output matrix
				 @return bool Label that indicates if the computation of the matrices is successful
				 */
				virtual bool computeDynamicModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C);


			private:
				

		}; //@class ArDrone
	
	} //@namespace example_models

} //@namespace mpc


#endif



