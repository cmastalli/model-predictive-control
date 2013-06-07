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
			 virtual void computeMPC(Eigen::VectorXd x_k, Eigen::VectorXd x_ref) = 0;

		    protected:


		    private:

      


    }; //@class Optimizer

    } //@namepace optimizer

}; //@namespace mpc

#endif

