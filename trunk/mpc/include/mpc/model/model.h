#ifndef MPC_MODEL_MODEL_H
#define MPC_MODEL_MODEL_H

#include <Eigen/Dense>

/**
 *  \addtogroup mpc
 *  @{
 */

//! Model Predictives Control interfaces and implementations
namespace mpc
{
	/**
	 *  \addtogroup model
	 *  @{
	 */
	 //! Model interfaces and implementations
    namespace model
    {
		/**
		 @brief This is the abstract class used to create and define different process models, in a state space representation. The models can be defined as Linear Time Invariant (LTI) such as 
		 \f{eqnarray*}{
			 \dot{x}(t) = Ax(t) + Bu(t) \\
			 y(t) = Cx(t)
   	     \f}
		 or Linear Time Variant (LTV) such as
		\f{eqnarray*}{
			 \dot{x}(t) = A(t)x(t) + B(t)u(t) \\
			 y(t) = C(t)x(t)
   	     \f}
		A boolean member variable defines the type of model used. The C matrix is not considered in the computation of the system matrices because this matrix is not used by the MPC algorithm, in an effort to reduce computation time.
		 */
		class Model
		{
			public:
	        	/** @brief Constructor function	*/
				Model() {};

				/** @brief Destructor function */
				~Model() {};

				/**
			 	@brief After the MPC makes an iteration, this function is used to set the current state as the new linearization points for a LTV model into global variables. 
			 	@param double* op_states 		new linearization point for the state vector
			 	*/
				virtual void setLinearizationPoints(double* op_states) = 0;
			
				/**
				 @brief Function to compute the matrices for a Linear model process. If the model is a LTI model, the function can be just defined to set the values of the model matrices.
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

				/** @brief Function to identify if the model is LTI or LTV 
					@return bool True if the model is LTV
				*/
				virtual bool getModelType() const;

				/** @brief Function that returns the current value of the operation points for the states */
				virtual double* getOperationPointsStates() const;

				/** @brief Function that returns the current value of the operation points for the inputs */
				virtual double* getOperationPointsInputs() const;


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

				/** @brief Boolean to check if the model is time variant or not (True = LTV) */
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

inline double* mpc::model::Model::getOperationPointsStates() const
{
	return op_point_states_;
}

inline double* mpc::model::Model::getOperationPointsInputs() const
{
	return op_point_input_;
}

#endif

