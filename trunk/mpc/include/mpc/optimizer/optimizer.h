#ifndef MPC_OPTIMIZER_OPTIMIZER_H
#define MPC_OPTIMIZER_OPTIMIZER_H

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
			 Optimizer();

		/**
		 @brief Destructor function
		 */
			~Optimizer();
	
                /**
		 @brief Function to perform the manipulation of the model parameters to be ready for the solver
		 @param mpc::Model *model pointer to the process model class "Model"
		 */

         virtual void setOptimizationParams(mpc::model::Model *model, MatrixXd &H_, MatrixXd &F_);

		/**
		 @brief Function to define the cost function associated to the MPC problem 
		 @param mpc::Model *model pointer to the process model class "Model"
		 @param int &nWSR number of working set recalculations
		 @param double *cputime pointer to the defined time to solve the optimization problem. If NULL, it provides on output the actual calculation time of the optimization problem.
		 */
		 virtual void computeOpt(MatrixXd &H_, MatrixXd &F_, int &nWSR, double *cputime); // TODO include the matrices from the constraints
																						  // 	  as arguments as well
		 int &nWSR;	//number of working set recalculations

	    protected:



	    private:

	        


    }; //@class Optimizer

    } //@namepace optimizer

}; //@namespace mpc


