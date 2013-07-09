#ifndef MPC_OPTIMIZER_OPTIMIZER_H
#define MPC_OPTIMIZER_OPTIMIZER_H


#include <mpc/model/model.h>

namespace mpc
{

    namespace optimizer
    {
		/**
		 @class Optimizer
		 @brief Abstract class to define the optimization algorithm for Model Predictive Control
	 	 This class gives an abstract definition of the optimization problem in the context of Model Predictive Control
	 	 	\f{eqnarray*}{
			\mbox{Minimize} \; F(x) && \\
			\mbox{subject to} \; G(x) & = & 0 \\
			H(x) & \geq & 0
			\f}
	 	*/
		class Optimizer		
		{
            public:
				/** @brief Constructor function */
				Optimizer() {};
				
				/** @brief Destructor function */
				~Optimizer() {};
				
				/** @brief Function to define the initialization of optimizer */
				virtual bool init() = 0;
				
				/**
				 @brief Function to compute the optimization algorithm associated to the MPC problem 
				 @param double* H Hessian matrix
				 @param double* g Gradient vector
 				 @param double* G	Constraint matrix
				 @param double* lb	Low bound vector
				 @param double* ub	Upper bound vector
				 @param double* lbG	Low constraint vector
				 @param double* lbG	Upper constraint vector
				 @param double* cputime	CPU-time for computing the optimization. If NULL, it provides on output the actual calculation time of the optimization problem.
				 */
				virtual bool computeOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbG, double *ubG, double cputime) = 0;
				
				/** @brief Get the optimal solution vector when it was solve the optimization problem */
				virtual double* getOptimalSolution() = 0;
				
				/** @brief Get the number of constraints */
				virtual int getConstraintNumber() const;
				
				/** @brief Get the number of variables, i.e inputs * horizon */
				virtual int getVariableNumber() const;
				
				/** @brief Set the horizon of the MPC */
				virtual void setHorizon(int horizon);
				
				/** @brief Set the number of variables, i.e inputs * horizon */
				virtual void setVariableNumber(int variables);
				
				/** @brief Number of variables, i.e inputs * horizon */
   				int variables_;
   				
   				/** @brief Number of constraints */
   				int constraints_;
   				
   				/** @brief Horizon of MPC */
   				int horizon_;
   			
   			
			protected:
			


		    private:


	    }; //@class Optimizer

    } //@namepace optimizer

}; //@namespace mpc


inline int mpc::optimizer::Optimizer::getConstraintNumber() const
{
	return constraints_;
}

inline int mpc::optimizer::Optimizer::getVariableNumber() const
{
	return variables_;
}

inline void mpc::optimizer::Optimizer::setHorizon(int horizon)
{
	horizon_ = horizon;
}

inline void mpc::optimizer::Optimizer::setVariableNumber(int variables)
{
	variables_ = variables;
}

#endif

