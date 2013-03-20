#ifndef MPC_MODEL_MODEL_H
#define MPC_MODEL_MODEL_H


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
		virtual Model();

		/**
		 @brief Destructor function
		 */
		virtual ~Model();

		/**
		 @brief Function that defines the behavior of the process model
		 @param t Time instant
		 @param x State vector
		 @param u Control vector
		 @param x_dot Values of the derivatives
		 */
		virtual void dynamicFunction(double t, double *x, double *u, double *x_dot);
	
		/**
		 @brief Function to define the cost function associated to the MPC problem 
		 @param H
		 @param z
		 @param g
		 */
		virtual void setCostFunction();

		/**
		 @brief Function to set the whole optimization problem according to the documentation presented by qpOASES
		 @param
		 @param
		 */
		virtual void setOptProblem();
 

            protected:



            private:


	}; //@class ModelPredictiveControl

    } //@namespace model

} //@namespace mpc


