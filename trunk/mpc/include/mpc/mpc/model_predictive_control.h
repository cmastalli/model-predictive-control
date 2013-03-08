#ifndef OPENMPC_MPC_MODELPREDICTIVECONTROL_H
#define OPENMPC_MPC_MODELPREDICTIVECONTROL_H

namespace openmpc
{

    namespace mpc
    {
	/** \class ModelPredictiveControl
	The aim of this class is to solve the following optimal control problem with infinite optimization horizon for a given initial value \f$ x(t_0) = x_0. \f$
	\f{eqnarray*}{
	    \mbox{Minimize} \; J_\infty(x, u) & = & \sum\limits_{i=0}^{\infty} \int\limits_{t_i}^{t_{i + 1}} L \left( \tau, x_{u}(\tau, x_0), u(\tau, x_0) \right) d \tau + \sum\limits_{i=0}^{\infty} l \left( t_k, x_{u}(t_i, x_0), u(t_i, x_0) \right) \\
	    \dot{x}_{u}(t) & = & f(x_{u}(t, x(t_k)), u(x(t_k), t)) \qquad \forall t \in [t_0, \infty) \\
	    x_{u}(0, x_0) & = & x_0 \\
	    x_{u}(t, x_0) & \in & X \qquad \forall t \in [t_0, \infty] \\
	    u(t, x_0) & \in & U \qquad \forall t \in [t_0, \infty)
	    \f}
	Since solving this problem usually requires the solution of a Hamilton-Jacobi-Bellman (partial) differential equation, we use a receding horizon control approach an approximate the infinite horizon solution by the solution of a sequence of finite horizon optimal control problems:
	\f{eqnarray*}{
	    \mbox{Find} \; \mu(x(t_k)) & := & u_{[0]} \\
	    \mbox{ST.} \;\; u_{[0, N-1]} & = & \mbox{{\it argmin}}_{u \in \mathcal{U}_N} J_N (x(t_k), u) \\
	    J_N (x(t_k), u) & = & \sum\limits_{i=0}^{N - 1} \int\limits_{t_i^k}^{t_{i + 1}^k} L \left( \tau, x_{u}(\tau, x(t_k)), u(\tau, x(t_k)) \right) d \tau \\
	    && + \sum\limits_{i=0}^{N - 1} l \left( t_i^k, x_{u}(t_i^k, x(t_k)), u(t_i^k, x(t_k)) \right) + F(t_N^k, x_{u}(t_N^k, x(t_k))) \\
	    \dot{x}_{u}(t) & = & f(t, x_{u}(t, x(t_k)), u(x(t_k), t)) \qquad \forall t \in [t_0^k, t_N^k] \\
	    x_{u}(0, x(t_k)) & = & x(t_k) \\
	    x_{u}(t, x(t_k)) & \in & X \qquad \forall t \in [t_0^k, t_N^k] \\
	    u(x(t_k), t) & \in & U \qquad \forall t \in [t_i^k, t_{i + 1}^k)
	    \f}
	To solve each of these optimal control problems the function mpc::ModelPredictiveControl::initCalc discretizes the control problem with a not necessarily equidistant time grid. The resulting optimization problem is then solved by a (predefined) minimization routine.\n
	Then the first value of the computed control is implemented and the optimization horizon is shifted forward in time. This allows the procedure to be applied iteratively and computes a (suboptimal) infinite horizon control. \n
	Note that the function mpc::ModelPredictiveControl::shiftHorizon can be used to shift the optimization horizon forward in time by one or more sampling intervals. The required initial value \f$ x(t_{j+1}) \f$, however, has to be supplied by an external routine. This corresponds to the usage of this method in real life since the internal model and the external plant not necessarily coincide. \n
	@brief Class for solving the model predictive control problem
	*/
	class ModelPredictiveControl
	{
	    public:
		/**
		@brief Constructor
		@param INFTY Defines the value \f$ \infty \f$
		*/
		virtual ModelPredictiveControl(double infty = 1E19);

		/**
		@brief Destructor
		*/
		virtual ~ModelPredictiveControl();

		/**
		@brief Function to initialize the MPC object and set all relevant internal data
		@param odemanager Pointer to used class yane::MPC::OdeManager object
		@param minimizer Pointer to used class yane::MinProg::NLP object
		@param model Pointer to used class yane::Model::Model object
		@param horizon Maximal length of the optimization horizon
		@param precedesteps Number of prediction steps to obtain initial value for optimization
		@param shootingdata Pointer to used class yane::MPC::ModelShootingData object
		@throw yane::MinProg::MinProgException Is thrown if problem cannot be initialized
		@throw yane::Utils::ValueException Is thrown if input values are incorrect
		@throw yane::Utils::Exception Is thrown if called objects reveal error messages
		@throw yane::Utils::MemoryException Is thrown if memory allocation failed
		*/
		virtual void reset(OdeManager * odemanager, Optimizer::XXX *optimizer, mpc::Model *model, int horizon, int precedesteps = 0, ModelShootingData *shootingdata = 0);

		/**
		@brief Function to allocate the memory for the time and the control values
		@param t Time grid
		@param u Control vector
		@throw yane::Utils::NotInitializedException Is thrown if the yane::MPC::MPC object is not yet defined, i.e. if the yane::MPC::MPC::reset function has not been called
		@throw yane::Utils::MemoryException Is thrown if memory allocation failed
		*/
		virtual void allocateMemory(double* & t, double* &u );

		/**
		@brief Function to assign equidistant values to the time grid variable
		@param t Time grid
		@param t_start Initial time
		@param h_new Sampling width
		@throw yane::Utils::NotInitializedException Is thrown if the yane::MPC::MPC object is not yet defined, i.e. if the yane::MPC::MPC::reset function has not been called
		*/
		virtual void generateTimeGrid(double *t, double t_start, double h_new);

		/**
		@brief Function to initialize the MPC problem
		@param t Time grid
		@param u0 Initial guess of the control on the optimization horizon
		@throw yane::Utils::NotInitializedException Is thrown if the yane::MPC::MPC object is not yet defined, i.e. if the yane::MPC::MPC::reset function has not been called
		@throw yane::Utils::ValueException Is thrown if a null pointer submitted
		*/
		virtual void initMPCProblem(double * t, double * u0);

		/**
		@brief Function to start the computation of the MPC solution
		@param x Estimate of the initial value of the state
		@param realtimesteps Number of time steps on the time grid after which the computation shall be terminated (0 = no limit)
		@param aborttimeoffset Time offset after which the computation shall be terminated (negative values shorten the time interval available for optimization)
		@param abortSuboptimalityDegree Bound on the degree of suboptimality to stop the optimization
		@throw yane::Utils::NotInitializedException Is thrown if the yane::MPC::MPC object is not yet defined, i.e. if the yane::MPC::MPC::reset function has not been called
		@throw yane::MinProg::SolverWarning Is thrown if the yane::MinProg::NLP object terminates with a warning
		@throw yane::OdeSolve::OdeSolEx Is thrown if the yane::OdeSolve::OdeSolve object terminates with an error
		@throw yane::Utils::ValueException Is thrown if a null pointer is submitted
		@throw yane::Utils::Exception Is thrown if any other error occurs
		*/
		virtual void startMPCSolution(double *x, int realtimesteps = 0, double aborttimeoffset = 0.0, double abortSuboptimalityDegree = INFINITY);


		/**
		@brief Function to modify the length of the optimization horizon
		@param new_horizon New length of the optimization horizon
		@param h_new Length of the sampling width for added time instances
		@throw yane::Utils::ValueException Is thrown if the new horizon length is larger than the maximal horizon length or if the new horizon length is negative
		@throw yane::Utils::NotInitializedException Is thrown if the yane::MPC::MPC object is not yet defined, i.e. if the yane::MPC::MPC::reset function has not been called
		@throw yane::MinProg::MinProgException Is thrown if the discretized optimization problem or the prediction cannot be initialized
		@throw yane::Utils::MemoryException Is thrown if memory allocation fails
		@throw yane::Utils::ValueException Is thrown if a null pointer is submitted
		@throw yane::Utils::Exception Is thrown if an other error occurs
		*/
		virtual void resizeHorizon(int new_horizon, double h_new);

		/**
		@brief Function to shift the time horizon of the optimal control problem
		@param next_controls Pointer to the control values which are to be implemented next
		@param next_timesteps Pointer to the time instances for which the next control values are computed
		@param m Number of sampling instances to be shifted (default 1)
		@param x Estimate of the current state vector
		@param h_new Length of the added sampling interval at the end of the shifted optimization horizon
		@throw yane::Utils::NotInitializedException Is thrown if the yane::MPC::MPC object is not yet defined, i.e. if the yane::MPC::MPC::reset function has not been called
		@throw yane::MinProg::MinProgException Is thrown if the discretized optimization problem or the prediction cannot be initialized
		@throw yane::OdeSolve::OdeSolEx Is thrown if the yane::OdeSolve::OdeSolve object terminates with an error
		@throw yane::Utils::MemoryException Is thrown if memory allocation fails
		@throw yane::Utils::ValueException Is thrown if input is errorneous
		@throw yane::Utils::Exception Is thrown if an other error occurs
		*/
		virtual void shiftHorizon(double *next_controls, double *next_timesteps, int m = 1, double *x = 0, double h_new = 0.0);

		/**
		@brief Function to configure the yane::MPC::MPC::shiftHorizon function
		@param optimize Decision variable on the type of shift
		- (0): without optimization
		- (1): optimization of the \f$ m \f$ new control vectors
		- (2): for \f$ i \in \{ 0, \ldots, m - 1\} \f$ a repreated implementation of the in each step new first control vector and a shift of the horizon by \f$ 1 \f$ is executed where the last \f$ N - m + i \f$ control vectors in each step \f$ i \in \{ 0, \ldots, m - 2\} \f$ are reoptimized
		@throw yane::Utils::NotInitializedException Is thrown if the yane::MPC::MPC object is not yet defined, i.e. if the yane::MPC::MPC::reset function has not been called
		@throw yane::Utils::ValueException Is thrown if input is errorneous
		*/
		virtual void setConfigShiftHorizon(T_SHIFTSTATE optimize = STANDARD, T_SHIFTRETURNDATA returndata = PREDICTED_DATA);

		/**
		@brief Function to modify the absolute and relative error tolerances of the internally used mpc::Model::Simulator object
		@param rtol Relative tolerance
		@param atol Absolute tolerance
		@throw yane::Utils::MemoryException Is thrown if memory allocation fails
		@throw yane::Utils::Exception Is thrown if an other error occurs
		*/
		virtual void setInternalSimulatorPrecision(double rtol, double atol);

		/**
		@brief Function to modify the absolute and relative error tolerance vectors of the internally used mpc::Model::Simulator object
		@param rtol Vector of relative tolerances
		@param atol Vector of absolute tolerances
		@throw yane::Utils::MemoryException Is thrown if memory allocation fails
		@throw yane::Utils::Exception Is thrown if an other error occurs
		*/
		virtual void setInternalSimulatorPrecision(double *rtol, double *atol);

		/**
		@brief Function to set the decision variable on showing all errors of the used mpc::Optimizacion::XXX object
		@param showSqpError Decision variable on showing all errors of the used mpc::Optimization::XXX object
		*/
		virtual void setShowSqpError(bool showSqpError);

		/**
		@brief Function to return the openmpc::model::Model object used for optimization and prediction
		@return Pointer the used openmpc::model::Model object
		@throw yane::Utils::NotInitializedException Is thrown if the mpc::ModelPredictiveControl object is not yet defined, i.e. if the mpc::ModelPredictiveControl::reset function has not been called
		*/
		openmpc::model::Model *model();

		/**
		@brief Function to return the maximal allowable optimization horizon
		@return Maximal allowable optimization horizon
		@throw yane::Utils::NotInitializedException Is thrown if the yane::MPC::MPC object is not yet defined, i.e. if the mpc::ModelPredictiveControl::reset function has not been called
		*/
		int getMaxHorizon();

		/**
		@brief Function to return the current optimization horizon
		@return Current optimization horizon
		@throw yane::Utils::NotInitializedException Is thrown if the mpc::ModelPredictiveControl object is not yet defined, i.e. if the mpc::ModelPredictiveControl::reset function has not been called
		*/
		int getHorizon();

		/**
		@brief Function to return the current number of precedesteps
		@return Current number of precedesteps
		@throw yane::Utils::NotInitializedException Is thrown if the mpc::ModelPredictiveControl object is not yet defined, i.e. if the mpc::ModelPredictiveControl::reset function has not been called
		*/
		int getPrecedesteps();

		/**
		@brief Function to return the current state of the internal optimization error variable
		@return Value of the internal optimization error variable
		*/
		T_OPTIMALITYSTATE optimalityState();

		/**
		@brief Function to return the pointer of the discretized optimal control problem
		@return Pointer of the discretized optimal control problem
		*/
		openmpc::MPCproblem *problem();

		/**
		@brief Function to return the pointer of the simulator used within the MPC routine
		@return Pointer of the simulator used within the MPC routine
		*/
		openmpc::Simulator *modelSim();


	    protected:
		/**
		@brief Function to free allocated memory
		*/
		virtual void freeMemory();

		/**
		@brief Function to set he configuration of the suboptimality based stopping criterion
		@param x Current state vector
		@throw yane::Utils::NotInitializedException Is thrown if the yane::MPC::MPC object is not yet defined, i.e. if the yane::MPC::MPC::reset function has not been called
		*/
		virtual void setAbortSuboptimalityValues(double *x);

		MPCProblem *problem_;  /** @brief Pointer to used class yane::MPC::Discretization object */
		MPCProblem *problem_shift_;  /** @brief Pointer to used class yane::MPC::Discretization object used within the yane::MPC::MPC::shiftHorizon method */

		openmpc::Optimization::XXX *minimizer_; /** @brief Pointer to used class yane::MinProg::NLP object */
		openmpc::OdeManager *odemanager_;  /** @brief Pointer to used class yane::MPC::OdeManager object */
		openmpc::Model *model_;  /** @brief Pointer to used class yane::Model::Model object */

		openmpc::Model::ModelShootingData *shootingdata_; /** @brief Pointer to used class yane::MPC::ModelShootingData object */

		openmpc::Model::Simulator *modelsim_; /** @brief Pointer to used class yane::Model::Simulator object */

		int horizon_;    /** @brief Length of the optimization horizon */

		int maxhorizon_;    /** @brief Maximal length of the optimization horizon */

		int precedesteps_;    /** @brief Number of prediction steps to obtain initial value for optimization */

		double infty_;    /** @brief Value for \f$ \infty \f$ */

		double *timesteps_;   /** @brief Time grid */

		double *timesteps_preceded_;  /** @brief Time grid for prediction steps */

		double *u_;     /** @brief Control vector */

		double *u_preceded_;   /** @brief Control vector for prediction steps */

		double *x_simulated_;   /** @brief Simulated state vector at the first time instance of optimization */

		T_OPTIMALITYSTATE optimalitystate_; /** @brief Value of the internal optimization error variable */

		bool showsqperror_;   /** @brief Decision variable on showing all errors of the used mpc::Optimization::XXX object */

		double abortsuboptimalitydegree_; /** @brief Bound on the degree of suboptimality to stop the optimization */

		T_SHIFTSTATE optimize_;    /** @brief Decision variable on the type of shift
		- STANDARD: without optimization
		- ONESTEP: optimization of the \f$ m \f$ new control vectors
		- REPEATED: for \f$ i \in \{ 0, \ldots, m - 1\} \f$ a repreated implementation of the in each step new first control vector and a shift of the horizon by \f$ 1 \f$ is executed where the last \f$ N - m + i \f$ control vectors in each step \f$ i \in \{ 0, \ldots, m - 2\} \f$ are reoptimized */

		T_SHIFTRETURNDATA returndata_;	/** @brief Decision variable on the return data of shifted
		- CURRENT_DATA: data of time grid and control at current time instant is returned
		- PREDICTED_DATA: data of time grid and control at initial time of the underlying optimal control problem is returned, i.e. predicted data */


	    private:
		mpc::OdeSolve::OdeConfig *modelsimconfig_;	/** @brief Pointer to use yane::OdeSolve::OdeConfig object used for forward prediction */

		double *rtolvec_;	/** @brief Vector of relative error tolerances */

		double *atolvec_;	/** @brief Vector of absolute error tolerances */


	};  //@class ModelPredictiveControl

    }  //@namespace mpc

}  //@namespace openmpc


#endif //OPENMPC_MPC_MODELPREDICTIVECONTROL_H
