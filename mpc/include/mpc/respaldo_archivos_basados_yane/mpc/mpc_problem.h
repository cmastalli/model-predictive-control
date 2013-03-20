#ifndef OPENMPC_MPC_DISCRETIZATION_H
#define OPENMPC_MPC_DISCRETIZATION_H

#include "typedefs.h"
//#include "../minprog.h"

#include <cmath>



namespace openmpc
{
    namespace mpc
    {
	/** \class Discretization
	This class can be used to define and solve an optimal control problem of the form
	\f{eqnarray*}{
	\mbox{Minimize} \; J_N(x, u) & = & \sum\limits_{i = 0}^{N - 1} \int\limits_{t_i}^{t_{i+1}} l(x, u) dt + F(x(t_N)) \\
	\mbox{subject to} & \\
	\dot{x}(t) & = & f(x(t), u(t_i)) \qquad \forall t \in [t_i, t_{i + 1}) \\
	g_l(t) & \leq & g(x(t), u(t)) \leq g_u(t) \qquad \forall t \in [t_0, t_N] \\
	x(\cdot) & \in & X \\
	u(\cdot) & \in & U
	\f}
	To this end a (not necessarily equidistant) time grid is defined and the control problem is discretized with respect to this time grid. The resulting optimization problem is then handed over to a class yane::MinProg::NLP object.
	\brief Class for discretizing optimal control problem
	 */
	class MPCProblem
	{
	    public:
		/**
		 Constructor
		 @param odemanager Pointer of the odemanager object of class yane::MPC::OdeManager
		 @param minimizer Pointer of the minimizer object of class yane::MinProg::NLP
		 @param model Pointer of the model object of class yane::Model::Model
		 @param horizon Length of the optimization horizon in sampling instances
		 @param INFTY Value for infinity
		 @throw yane::Utils::ValueException Is thrown if input values are incorrect
		 @throw yane::Utils::MemoryException Is thrown if memory allocation failed
		 @throw yane::MinProg::MinProgException Is thrown if the discretized optimization problem or the prediction or the odemanager cannot be initialized
		 @throw yane::Utils::Exception Is thrown if the odemanager cannot be initialized
		 @brief Constructor
		 */
		MPCProblem(OdeManager *odemanager, yane::MinProg::NLP *minimizer, yane::Model::Model *model, int horizon, ModelShootingData *shootingdata, double INFTY=1E19);

		/**
		 @brief Destructor
		 */
		~MPCProblem();

		/**
		Function to initialize the discretized optimal control problem
		 @param t Time grid
		 @param u0 Initial guess of the control
		 @brief Function to initialize the discretized optimal control problem
		 */
		void initCalc(double *t, double *u0);

		/**
		Function to solve the discretized optimal control problem
		 @param x Estimate of the initial value of the state
		 @param realtimesteps Number of time steps on the time grid after which the computation shall be terminated (0 = no limit)
		 @param aborttimeoffset Time offset after which the computation shall be terminated (negative values shorten the time interval available for optimization)
		 @throw yane::Utils::ValueException Is thrown if a null pointer is submitted
		 @throw yane::MinProg::SolverWarning Is thrown if the yane::MinProg::NLP object terminates with a warning
		 @throw yane::Utils::Exception Is thrown if any other error occurs
		 @brief Function to solve the discretized optimal control problem
		 */
		void calc(double *x, int realtimesteps = 0, double aborttimeoffset = 0.0);

		/**
		Function to create a cloned object using identical odemanager and minimizer from classes yane::MPC:OdeManager and yane::MinProg::NLP respectively. The clone can be initialized with a different horizon length. \n
		Note that local modifications of the odemanager and the minimizer objects are not cloned and have to be redone. This object must be destructed by the user and its destruction automatically destructs the corresponding odemanager and minimizer objects.
		 @param horizon Length of the horizon for the cloned object
		 @throw yane::MPC::MPCException Is thrown if the underlying yane::MPC::MPC object terminates with an error
		 @throw yane::Utils::MemoryException Is thrown if memory allocation fails
		 @throw yane::Utils::Exception Is thrown if any other error occurs
		 @brief Function to create a cloned object using identical odemanager and minimizer from classes yane::MPC:OdeManager and yane::MinProg::NLP respectively
		 */
		MPCProblem *clone(int horizon = -1);

		/**
		Function to return the pointer of the used class yane::MPC::OdeManager object
		 @return Pointer of the used class yane::MPC::OdeManager object
		 @brief unction to return the pointer of the used class yane::MPC::OdeManager object
		 */
		OdeManager *odeManager();

		/**
		Function to return the pointer of the used class openmpc::Optimizer::NLP object
		 @return Pointer of the used class openmpc::Optimizer::NLP object
		 @brief Function to return the pointer of the used class yane::MinProg::NLP object
		 */
		openmpc::Optimizer::NLP *optimizer();

		/**
		Function to return the current horizon length
		 @return Current horizon length
		 @brief Function to return the current horizon length
		 */
		int horizon();

		/**
		Function to set the data used for the suboptimality based stopping criterion
		 @param abortSuboptimalityDegree Bound of the suboptimality degree
		 @param previousCostfunctionValue Value of the cost function in the previous MPC step
		 @param previousRunningcostValue Value of the running cost in the previous MPC step
		 @brief Function to set the data used for the suboptimality based stopping criterion
		 */
		void setAbortSuboptimalityCriterionValues(double abortSuboptimalityDegree, double previousCostfunctionValue = 0.0, double previousRunningcostValue = 0.0);

		/**
		Function to return the current value of the suboptimality bound used within the suboptimality based stopping criterion
		 @return Current value of the suboptimality bound
		 @brief Function to return the current value of the suboptimality bound used within the suboptimality based stopping criterion
		 */
		double abortSuboptimalityDegree();

		/**
		Function to evaluate the cost function
		 @param x Estimate of the initial value of the state
		 @param fx Value of the cost function
		 @param params Parameter vector of the optimal control problem
		 @brief Function to evaluate the cost function
		 */
		static void minFunc(const double *x, double *fx, void *params);

		/**
		Function to evaluate the restrictions
		 @param x Estimate of the initial value of the state
		 @param fx Vector of values of the restrictions
		 @param params Parameter vector of the optimal control problem
		 @brief Function to evaluate the restrictions
		 */
		static void minRestr(const double *x, double *fx, void *params);

		/**
		Function to evaluate the gradient of the cost function
		 @param x Estimate of the initial value of the state
		 @param fx Gradient of the cost function
		 @param params Parameter vector of the optimal control problem
		 @brief Function to evaluate the gradient of the cost function
		 */
		static void minFuncGrad(const double *x, double *fx, void *params);

		/**
		Function to evaluate the Jacobian of the restrictions
		 @param x Estimate of the initial value of the state
		 @param fx Jacobian of the constraints
		 @param params Parameter vector of the optimal control problem
		 @brief Function to evaluate the Jacobian of the restrictions
		 */
		static void minRestrGrad(const double *x, double *fx, void *params);

		/**
		Function to check whether the suboptimality based stopping criterion is satisfied
		 @param currentCostfunctionValue Value of the cost function for the current control vector
		 @return Decision variable
		 @brief Function to check whether the suboptimality based stopping criterion is satisfied
		 */
		bool suboptimalityCheck(double currentCostfunctionValue);

		/**
		Function to return the total number of restrictions of the discretized optimal control problem
		 @return Total number of restrictions of the discretized optimal control problem
		 @brief Function to return the total number of restrictions of the discretized optimal control problem
		 */
		int totalRestrictionNumber();

		/**
		Function to return the pointer of the differential equation manager
		 @return Pointer of the differential equation manager
		 @brief Function to return the pointer of the differential equation manager
		 */
		OdeManager *odemanager();

		/**
		Function to return the structure of shooting nodes
		 @return Structure of the shooting nodes
		 @brief Function to return the structure of the shooting nodes
		 */
		T_SHOOTINGDATA *sdata();

		/**
		Function to return the number of shooting nodes
		 @return Number of shooting nodes
		 @brief Function to return the number of shooting nodes
		 */
		int sdataLength();

		/**
		Function to return the pointer of the model
		 @return Pointer of the model
		 @brief Function to return the pointer of the model
		 */
		openmpc::model::Model *model();

		/**
		Function to return the dimension of the control
		 @return Dimension of the control
		 @brief Function to return the dimension of the conrol
		 */
		int getDimensionControlSignal();

		/**
		Function to return the dimension of the state variable
		 @return Dimension of the state variable
		 @brief Function to return the dimension of the state variable
		 */
		int getDimensionState();

		/**
		Function to return the number of restrictions of the optimal control problem before discretization
		 @return Number of restrictions of the optimal control problem before discretization
		 @brief Function to return the number of restrictions of the optimal control problem before discretization
		 */
		int getDimensionRestrictions();

		/**
		Function to return the number associated with infinity
		 @return Number associated with infinity
		 @brief Function to return the number associated with infinity
		 */
		double infty();

		/**
		Function to return the pointer of the lower bounds of the states
		 @return Pointer of the lower bounds of the states
		 @brief Function to return the pointer of the lower bounds of the states
		 */
		double *lbound_x();

		/**
		Function to return the pointer of the upper bounds of the states
		 @return Pointer of the upper bounds of the states
		 @brief Function to return the pointer of the upper bounds of the states
		 */
		double *ubound_x();

		/**
		Function to return the pointer of the lower bounds of the control
		 @return Pointer of the lower bounds of the control
		 @brief Function to return the pointer of the lower bounds of the control
		 */
		double *lbound_u();

		/**
		Function to return the pointer of the upper bounds of the control
		 @return Pointer of the upper bounds of the control
		 @brief Function to return the pointer of the upper bounds of the control
		 */
		double *ubound_u();

		/**
		Function to return the pointer of zero control vector
		 @return Pointer of a zero control vector
		 @brief Function to return the pointer of a zero control vector
		 */
		double *u_zero();

		/**
		Function to return the pointer of the internal copy of the state vector
		 @return Pointer of the internal copy of the state vector
		 @brief Function to return the pointer of the internal copy of the state vector
		 */
		double *currentx();

		/**
		Function to return the pointer of the internal copy of the time grid
		 @return Pointer of the internal copy of the time grid
		 @brief Function to return the pointer of the internal copy of the time grid
		 */
		double *timesteps();

	    private:
		/**
		Function to generate and evaluate the box constraints of the state and control variables as well as the supplied nonlinear constraints
		 @param t Time grid
		 @param x Estimate of the initial value of the state
		 @param u Control vector
		 @param fx Vector of values of the restrictions
		 @param discretization Underlying discretized optimal control problem
		 @param configuration Underlying configuration of the optimal control problem
		 @brief Function to generate and evaluate the box constraints of the state and control variables as well as the supplied nonlinear constraints
		 */
		static void restriction(double *t, double *x, double *u, double *fx, void *discretization, void *configuration);

		OdeManager *odemanager_;		/**< @brief Pointer of the used class yane::MPC::OdeManager object */
		openmpc::Optimizer::NLP *optimizer_;	/**< @brief Pointer of the used class yane::MinProg::NLP object */
		openmpc::model::Model *model_;		/**< @brief Pointer of the used class yane::Model::Model object */
		ModelShootingData *shootingdata_;	/**< @brief Pointer of the used class yane::MPC::ModelShootingData object */

		yane::MinProg::T_MEMMODEL memmodel_;	/**< @brief Decision variable for memory model
								-# BYROW - Save data in rows (C)
								-# BYCOLUMN - Save data in columns (FORTRAN) */

		int dimension_x_;			/**< @brief Dimension of the state vector */
		int dimension_u_;			/**< @brief Dimension of the control vector */
		int dimension_restr_;			/**< @brief Dimension of the noninear constraints*/
		int horizon_;				/**< @brief Length of the horizon */

		int totalRestrictionNumber_;		/**< @brief Total number of constraints of the MPC problem */

		double *currentx_;			/**< @brief Internal copy of the state vector */
		double *timesteps_;			/**< @brief Internal copy of the time grid */
		double *u_;				/**< @brief Internal copy of the control vector */
		double *u_zero_;			/**< @brief Control vector containing only zero entries */

		double *lbound_x_;			/**< @brief Internal copy of the lower bounds of the state vector */
		double *ubound_x_;			/**< @brief Internal copy of the upper bounds of the state vector */

		double *lbound_u_;			/**< @brief Internal copy of the lower bounds of the control vector */
		double *ubound_u_;			/**< @brief Internal copy of the upper bounds of the control vector */

		T_SHOOTINGDATA *sdata_;			/**< @brief Structure of the shooting nodes */
		int sdatalength_;			/**< @brief Number of shooting nodes */

		double infty_;				/**< @brief Predefinition of infinity */

		bool iscloned_;				/**< @brief Decision variable whether the current class yane::Discretization::Discretization object is a clone */

		double previouscostfunctionvalue_;	/**< @brief Value of the cost function in the previous MPC step */
		double previousrunningcostvalue_;	/**< @brief Value of the running cost in the previous MPC step */
		double abortsuboptimalitydegree_;	/**< @brief Value of the suboptimality bound used for the suboptimality based stopping criterion */
	};  //@class Discretization

    }  //@namespace mpc

}  //@namespace openmpc

#endif  //OPENMPC_MPC_DISCRETIZATION_H
