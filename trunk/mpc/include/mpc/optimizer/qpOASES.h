#ifndef QPOASES_H
#define QPOASES_H

#include <mpc/optimizer/optimizer.h>
#include <ros/ros.h>
#include <qpOASES.hpp>

USING_NAMESPACE_QPOASES

namespace mpc
{
	namespace optimizer
	{
	
		class qpOASES : public mpc::optimizer::Optimizer
		{
			public:
				// Constructor
				qpOASES(ros::NodeHandle node_handle);

				//Destructor
				~qpOASES() {}


			   /**
				 @brief Function to solve the optimization problem formulated in the MPC  
				 @param Eigen::VectorXd x_k 		state vector
				 @param Eigen::VectorXd x_ref		reference vector 
				 */
				virtual bool initSolver(double *H, 
										double* g, 
										double *G, 
										double *lb, 
										double *ub, 
										double *lbA, 
										double *ubA, 
										int &nWSR, 
										double *cputime);

				virtual bool hotstartSolver(double *g_new, 
											double *G_new, 
											double *lb_new, 
											double *ub_new, 
											double *lbA_new, 
											double *ubA_new, 
											int &nWSR, 
											double *cputime,
											double **optSol);	// In here, the address of the solution array is passed (&optSol)

			protected:

				int nVar_, nConst_, horizon_;
				returnValue retval_;
				ros::NodeHandle nh_opt_;

			private:
				

				//int &nWSR;	//number of working set recalculations
				/* QProblem object which is used to solve the quadratic problem */
				QProblem * solver_;
				
		}; // @class qpOASES

	} // @namespace optimizer

} // @namespace mpc



#endif
