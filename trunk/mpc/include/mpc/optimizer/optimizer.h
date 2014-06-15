#ifndef MPC_OPTIMIZER_OPTIMIZER_H
#define MPC_OPTIMIZER_OPTIMIZER_H


#include <mpc/model/model.h>
/**
 *  \addtogroup mpc
 *  @{
 */

//! Model Predictives Control interfaces and implementations
namespace mpc
{
	/**
	 *  \addtogroup optimizer
	 *  @{
	 */
	 //! Optimizer interfaces and implementations
    namespace optimizer
    {
		/**
		 @class Optimizer
		 @brief Abstract class to define the optimization algorithm for Model Predictive Control. This class acts as an interface to use a defined optimization solver software as a part of this library in order to provide different solver options for the end user to solve the basic optimization problem that rises in MPC. As more solvers are adapted to this library with this class, more options to try different optimization methods are available to select the most suitable one depending on each case. The basic  
	 	 \f{eqnarray*}{
			\mbox{Minimize} \; F(x) && \\
			\mbox{subject to} \; G(x) & = & 0 \\
			H(x) & \geq & 0
		\f}
		 As more solvers are adapted to this library with this class, more options to try different optimization methods are available to select the most suitable one depending on each case.
	 	*/
		class Optimizer		
		{
            public:
				/** @brief Constructor function */
				Optimizer() {};
				
				/** @brief Destructor function */
				~Optimizer() {};
				
				/**
				 @brief Function to perform the initialization of optimizer, if this applies.
				 @return Label that indicates if the initialization of the optimizer is successful
				 */
				virtual bool init() = 0;
				
				/**
				 @brief Function to compute the optimization algorithm associated to the MPC problem. 
				 @param double* H Hessian matrix
				 @param double* g Gradient vector
 				 @param double* G	Constraint matrix
				 @param double* lb	Low bound vector
				 @param double* ub	Upper bound vector
				 @param double* lbG	Low constraint vector
				 @param double* lbG	Upper constraint vector
				 @param double cputime	CPU-time for computing the optimization. If NULL, it provides on output the actual calculation time of the optimization problem.
				 @return bool Label that indicates if the computation of the optimization is successful
				 */
				virtual bool computeOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbG, double *ubG, double cputime) = 0;
				
				/**
				 @brief Get the vector of optimal or sub-optimal solutions calculated by the mpc::optimizer::Optimizer::computeOpt() function (optimality of the function is defined by the solver that is adapted).
				 @return double* Optimal solution
				 */
				virtual double* getOptimalSolution() = 0;
				
				/**
				 @brief Get the number of constraints
				 @return int Number of constraints
				  */
				virtual int getConstraintNumber() const;
				
				/**
				 @brief Get the number of variables, i.e inputs * horizon
				 @return int Number of variables
				  */
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
/** @} End of Doxygen Groups*/


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

