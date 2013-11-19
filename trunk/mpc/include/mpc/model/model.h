#ifndef MPC_MODEL_MODEL_H
#define MPC_MODEL_MODEL_H

#include <Eigen/Dense>


namespace mpc
{
	/**
	 *  \addtogroup model
	 *  @{
	 */
	 //! Model interfaces and implementations
    namespace model
    {
        //TODO: define a cost function
		/**
		 @brief Abstract class to define the model of the process and the optimal control problem to be solved
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
		 */
		class Model
		{
			public:
	        	/** @brief Constructor function	*/
				Model() {};

				/** @brief Destructor function */
				~Model() {};

				/**
			 	@brief After the MPC makes an iteration, this function is used to set the new linearization points for a LTV model into 				global variables for the STDMPC class 
			 	@param double* op_states 		new linearization point for the state vector
			 	*/
				virtual void setLinearizationPoints(double* op_states) = 0;
			
				/**
				 @brief Function to compute the matrices for a Linear model process
				 @param Eigen::MatrixXd& A	State or System matrix
				 @param Eigen::MatrixXd& B	Input matrix
				 @return bool Label that indicates if the computation of the matrices is successful
				 */
				virtual bool computeLinearSystem(Eigen::MatrixXd& A, Eigen::MatrixXd& B) = 0;

				/** @brief Get the states number of the dynamic model */
				virtual int getStatesNumber() const;
				
				/** @brief Get the inputs number of the dynamic model */
				virtual int getInputsNumber() const;
				
				/** @brief Get the outputs number of the dynamic model */
				virtual int getOutputsNumber() const;

				virtual bool setStates(const double* states) const;	

				virtual bool setInputs(const double* inputs) const;	

				virtual bool getModelType() const;


            protected:
				/** @brief State matrix of the dynamic model */
				Eigen::MatrixXd A_;
				
				/** @brief Input matrix of the dynamic model */
				Eigen::MatrixXd B_;
				
				/** @brief Number of states of the dynamic model */
				int num_states_;
				
				/** @brief Number of inputs of the dynamic model */
				int num_inputs_;
				
				/** @brief Number of outputs of the dynamic model */
				int num_outputs_;
				
				/** Pointer to the array of the states operation points **/
				double* op_point_states_;

				/** Pointer to the array of the input operation points **/
				double* op_point_input_;

				/** @brief Boolean to check if the model is time variant or not */
				bool time_variant_;

            private:


		}; //@class ModelPredictiveControl

    } //@namespace model

} //@namespace mpc
/** @} End of Doxygen Groups*/

inline int mpc::model::Model::getStatesNumber() const
{
	return num_states_;
}

inline int mpc::model::Model::getInputsNumber() const
{
	return num_inputs_;
}

inline int mpc::model::Model::getOutputsNumber() const
{
	return num_outputs_;
}

inline bool mpc::model::Model::setStates(const double* states) const
{
	*op_point_states_ = *states;
	
	return true;
}

inline bool mpc::model::Model::setInputs(const double* inputs) const
{
	*op_point_input_ = *inputs;
	
	return true;
}

inline bool mpc::model::Model::getModelType() const
{	
	return time_variant_;
}

#endif

