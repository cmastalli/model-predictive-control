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
			 @brief Function to define the initialization of optimizer
			 */
			virtual bool init() = 0;

			/**
			 @brief Function to define the cost function associated to the MPC problem 
			 @param mpc::Model *model pointer to the process model class "Model"
			 @param int &nWSR number of working set recalculations
			 @param double *cputime pointer to the defined time to solve the optimization problem. If NULL, it provides on output the actual calculation time of the optimization problem.
			 */
			virtual bool computeOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbA, double *ubA, double cputime) = 0;

			//virtual bool hotstartSolver(double* g_new, double *lb_new, double *ub_new, double *lbA_new, double *ubA_new, int &nWSR, double *cputime, double *optimalSol) = 0;
		
			virtual double* getOptimalSolution() = 0;
			 
			virtual int getConstraintNumber() const;
								
			virtual int getVariableNumber() const;

			virtual void setHorizon(int horizon);

			virtual void setVariableNumber(int nVar);
			
   			int nVar_, nConst_, horizon_;
   			
   			
			protected:
			


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

inline void mpc::optimizer::Optimizer::setHorizon(int horizon)
{
	horizon_ = horizon;
}

inline void mpc::optimizer::Optimizer::setVariableNumber(int nVar)
{
	nVar_ = nVar;
}

#endif

