#ifndef MPC_OPTIMIZER_H
#define MPC_OPTIMIZER_H

namespace MPC
{

    class Optimizer		
    {
	public:
        
        // Constructor

        Optimizer();

        // Destructor

        ~Optimizer();
	

	/*

	@brief function to define the cost function associated to the MPC problem 
	@param MPC::Model *myModel		pointer to the process model class "Model"
	@param int &nWSR			number of working set recalculations
	@param double *cputime			pointer to the defined time to solve the optimization problem. If NULL, it provides on output 							the actual calculation time of the optimization problem.
	
	*/
        virtual void computeOpt(MPC::Model *myModel, int &nWSR, double *cputime);



        protected:


        private:


    }; // Model Predictive Control class

}; // mpc namespace
