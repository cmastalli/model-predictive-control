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
				 @brief Function to define the initialization of qpOASES optimizer
				 */
				virtual bool init();

			   /**
				 @brief Function to solve the optimization problem formulated in the MPC  
				 @param Eigen::VectorXd x_k 		state vector
				 @param Eigen::VectorXd x_ref		reference vector 
				 */
				virtual bool computeOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbA, double *ubA, double cputime);

				/*
				@brief When called, this function returns the optimal solution vector, optimalSol_
				*/
				double* getOptimalSolution();


			protected:

				double * optimal_solution_;
				returnValue retval_;


			private:
				ros::NodeHandle nh_;				

				//int &nWSR;	//number of working set recalculations
				/* SQProblem object which is used to solve the quadratic problem */
				SQProblem *solver_;
				
				bool qpOASES_initialized_;
				
				int nWSR_;
				
				
		}; // @class qpOASES

	} // @namespace optimizer

} // @namespace mpc



#endif
