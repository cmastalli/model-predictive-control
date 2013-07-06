#ifndef MPC_MODEL_MODEL_H
#define MPC_MODEL_MODEL_H

#include <Eigen/Dense>


namespace mpc
{

    namespace model
    {
        //TODO: define a cost function
	/**
	 This class gives an abstract definition of the process model and the optimal control problem which shall be considered. The model itself is defined via its dynamic
	 \f{eqnarray*}{
	 \dot{x}(t) = Ax(t) + Bu(t) \\
	 y(t) = Cx(t)
         \f}
	 on the optimization horizon \f$ [t_0, N] \f$ with initial value \f$ x(t_0, x_0) = x_0 \f$. Moreover, the solution of the control system shall satisfy given constraints
	 \f{eqnarray*}{
	 x(t, x_0) & \in & X \qquad \forall t \in [t_0, t_N] \\
	 u(t, x_0) & \in & U \qquad \forall t \in [t_k, t_k)
	 \f}
	Since we suppose at least control law \f$ u(\cdot) \f$ to exist which satisfies all these constraints, i.e. is feasible, an optimization critierion
	 \f{eqnarray*}{
	 J_N (x_0, u) & = & \sum\limits_{k=0}^{N - 1} \int\limits_{t_k}^{t_{k + 1}} L \left( \tau, x_{u}(\tau, x_0), u(\tau, x_0) \right) d \tau \\
	 && + \sum\limits_{k=0}^{N - 1} l \left( t_k, x_{u}(t_k, x_0), u(t_k, x_0) \right) + F(t_N, x_{u}(t_N, x_0))
	 \f}
	is added to measure the quality of feasible solutions.
	 @brief Abstract class to define the model of the process and the optimal control problem to be solved
	 */
		class Model
		{
			public:
	        	/**
	        	 @brief Constructor function
		 		 */
				Model() {};

				/**
				 @brief Destructor function
				 */
				~Model() {};
			
			
				virtual bool computeDynamicModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C) = 0;


				virtual int getStatesNumber() const;
				
				
				virtual int getInputsNumber() const;
				
				
				virtual int getOutputsNumber() const;

	
			/**
			 @brief Function to define the cost function associated to the MPC problem 
			 @param H
			 @param z
			 @param g
			 */
			/* virtual void setCostFunction();		This function is no longer required since the solver uses its own cost function defined in the documentation. */

			/**
			 @brief Function to set the whole optimization problem according to the documentation presented by qpOASES
			 @param
			 @param
			 */
			/* virtual void setOptProblem();		This function is not being implemented in the model interface, but in the optimizer interface instead. */
 

            protected:
				int states_, inputs_, outputs_;
				
				Eigen::MatrixXd A_, B_, C_;


            private:


		}; //@class ModelPredictiveControl

    } //@namespace model

} //@namespace mpc

inline int mpc::model::Model::getStatesNumber() const
{
	return states_;
}

inline int mpc::model::Model::getInputsNumber() const
{
	return inputs_;
}

inline int mpc::model::Model::getOutputsNumber() const
{
	return outputs_;
}

#endif

