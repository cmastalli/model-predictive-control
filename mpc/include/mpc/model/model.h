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
		Model();

		/**
		 @brief Destructor function
		 */
		~Model();

		/**
		 @brief Function that provides the model matrix A for each time instant for Linear Time Variant models. Polymorphism is used to 		implement this function for Linear Time Invariant models as well.
		 @param curent_time 	Time instant
		 @param &A 				Reference to the A matrix
		 */
		virtual void getModelParameterA(/*int current_time,*/Eigen::MatrixXd& A);

/**
		 @brief Function that provides the model matrix B for each time instant for Linear Time Variant models. Polymorphism is used to 		implement this function for Linear Time Invariant models as well.
		 @param curent_time 	Time instant
		 @param &B 				Reference to the B matrix
		 */
		virtual void getModelParameterB(/*int current_time,*/ Eigen::MatrixXd& B);

/**
		 @brief Function that provides the model matrix C for each time instant for Linear Time Variant models. Polymorphism is used to 		implement this function for Linear Time Invariant models as well.
		 @param curent_time 	Time instant
		 @param &C 				Reference to the C matrix 
		 */
		virtual void getModelParameterC(/*int current_time,*/ Eigen::MatrixXd& C);
	
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



            private:


	}; //@class ModelPredictiveControl

    } //@namespace model

} //@namespace mpc
#endif

