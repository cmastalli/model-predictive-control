#ifndef QPOASES_H
#define QPOASES_H

#include <mpc/optimizer/optimizer.h>
#include <ros/ros.h>
#include <qpOASES.hpp>

namespace mpc
{
	namespace optimizer
	{
	
		class qpOASES : public mpc::optimizer::Optimizer
		{
			public:
				// Constructor
				qpOASES() {};

				//Destructor
				~qpOASES() {};


			   /**
				 @brief Function to solve the optimization problem formulated in the MPC  
				 @param Eigen::VectorXd x_k 		state vector
				 @param Eigen::VectorXd x_ref		reference vector 
				 */
				virtual bool initSolver(int *nVar_,
										int *nConst_,
										double *H, 
										double* g, 
										double *G, 
										double *lb, 
										double *ub, 
										double *lbA, 
										double *ubA, 
										int &nWSR, 
										double *cputime);

				virtual void hotstartSolver(double *g_new, 
											double *G_new, 
											double *lb_new, 
											double *ub_new, 
											double *lbA_new, 
											double *ubA_new, 
											int &nWSR, 
											double *cputime,
											double &optSol_);


			private:
				

				//int &nWSR;	//number of working set recalculations
				/* QProblem object which is used to solve the quadratic problem */
				qpOASES::QProblem solver_;

				/* Array that stores the optimal solution for the quadratic problem */
				double * optSol_;
		}; // @class qpOASES

	} // @namespace optimizer

} // @namespace mpc

#endif
