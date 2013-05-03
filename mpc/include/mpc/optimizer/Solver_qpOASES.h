#ifndef SOLVER_QPOASES_H
#define SOLVER_QPOASES_H

#include <Eigen/Dense>
#include "optimizer.h"

namespace mpc
{
	namespace optimizer
	{
	
		class Solver_qpOASES : public mpc::optimizer::optimizer
		{


			public:
			
			// Constructor
			Solver_qpOASES();

			//Destructor
			~Solver_qpOASES();

			       /**
		 @brief Function to perform the manipulation of the model parameters to be ready for the qpOASES active set method solver.
		 @param mpc::Model *model 			pointer to the process model class of interest.
		 @param MatrixXd &H_				reference to a global variable that stores the matrix H of the optimization problem.
		 @param MatrixXd &F_				reference to a global variable that stores the matrix H of the 
		 */

		virtual void setOptimizationParams(mpc::model::Model *model, MatrixXd &H_, MatrixXd &F_);

				   /**
		 @brief Function to define the cost function associated to the MPC problem 
		 @param mpc::Model *model 			pointer to the process model class
		 @param int &nWSR 					number of working set recalculations
		 @param double *cputime 			pointer to the defined time to solve the optimization problem. If NULL, it provides on output the actual calculation time of the optimization problem. 
		 */

		virtual void computeOpt(MatrixXd &H_, MatrixXd &F_, int &nWSR, double *cputime);

			private:

		} // class Solver_qpOASES

	} // namespace optimizer

} // namespace mpc

