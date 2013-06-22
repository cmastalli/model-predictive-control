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
			virtual bool initSolver(double *H, double* g, double *G, double *lb, double *ub, double *lbA, double *ubA, int &nWSR, double *cputime) = 0;

			virtual bool hotstartSolver(double* g_new, double *G_new, double *lb_new, double *ub_new, double *lbA_new, double *ubA_new, int &nWSR, double *cputime, double **optSol) = 0;
			 
			virtual int getConstraintNumber() const;
								
			virtual int getVariableNumber() const; 

			virtual int getHorizon() const; 

		    
			protected:
			
			int nVar_, nConst_, horizon_;

		    private:


	    }; //@class Optimizer

    } //@namepace optimizer

}; //@namespace mpc


inline int mpc::optimizer::Optimizer::getConstraintNumber() const
{
	return nConst_;
}

inline int mpc::optimizer::Optimizer::getVariableNumber() const
{
	return nVar_;
}

inline int mpc::optimizer::Optimizer::getHorizon() const
{
	return horizon_;
}

#endif

