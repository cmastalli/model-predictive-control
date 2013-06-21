#ifndef MPC_OPTIMIZER_OPTIMIZER_H
#define MPC_OPTIMIZER_OPTIMIZER_H


#include <mpc/model/model.h>

namespace mpc
{

    namespace optimizer
    {

		/**
	 	@class Optimizer
	 	@brief Abstract class to define the optimization algorithm of model predictive control
	 	*/
		class Optimizer		
		{
            public:
        
        	/**
			 @brief Constructor function
	         */
			 Optimizer() {};

			/**
			 @brief Destructor function
			 */
			~Optimizer() {};


			/**
			 @brief Function to define the cost function associated to the MPC problem 
			 @param mpc::Model *model pointer to the process model class "Model"
			 @param int &nWSR number of working set recalculations
			 @param double *cputime pointer to the defined time to solve the optimization problem. If NULL, it provides on output the actual calculation time of the optimization problem.
			 */
			 virtual bool initSolver(int *nVar_, int *nConst_, double *H, double* g, double *G, double *lb, double *ub, double *lbA, double *ubA, int &nWSR, double *cputime) = 0;

			virtual bool hotstartSolver(double* g_new, double *G_new, double *lb_new, double *ub_new, double *lbA_new, double *ubA_new, int &nWSR, double *cputime, double &optSol) = 0;
			 
			 

		    protected:
			
			int states_, inputs_, outputs_, horizon_;

		    private:


	    }; //@class Optimizer

    } //@namepace optimizer

}; //@namespace mpc

#endif

