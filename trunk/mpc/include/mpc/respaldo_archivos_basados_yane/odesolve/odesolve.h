#ifndef MPC_ODESOLVER_ODESOLVE_H
#define MPC_ODESOLVER_ODESOLVE_H


namespace mpc
{
namespace odesolver
{
    class OdeSolve
    {
	public:
	    /**
	     @brief Destructor
	     */
	    virtual ~OdeSolve();

	    /**
	     @brief Function to initialize the object
	     @param func Pointer of the dynamic of the system
	     @param config Pointer of the configuration object
	     @throw yane::Utils::MemoryException Is thrown if the memory allocation failed
	     @throw WrongSolverInputException Is thrown if a null pointer is provided to the routine
	     @throw OdeSolEx Is thrown if an error occured during the allocation or configuration of the differential equation solver
	     @throw IncompatibleConfigException Is thrown if an error occured during the configuration of the differential equation solver
	     @sa OdeFunction
	     @sa resize
	     */
	    void reset(mpc::OdeFunction *func, mpc::OdeConfig *config);

	    /**
	     @brief Function to change the size of the dynamic of the system
	     @sa OdeFunction
	     @sa reset
	     @param dimension New size of the dynamic of the problem
	     @throw WrongSolverInputException Is thrown if the new size or the system is too large or less or equal to zero
	     @throw yane::Utils::NotInitializedException Is thrown if the solver has not been initialized
	     */
	    void resize(int dimension);

	    /**
	     @brief Function to refresh the configuration of the object
	     @throw yane::Utils::NotInitializedException Is thrown if the solver has not been initialized
	     */
	    void refreshConfig();


	    /**
	     @brief Function to compute the solution of the dynamic
	     @param t Terminal time for solving the differential or difference equation problem
	     @param rpar Array of double parameters
	     @param ipar Array of integer parameters
	     @throw noInitialValueException Is thrown if no initial value has been supplied
	     @throw yane::Utils::NotInitializedException Is thrown if the solver has not been initialized
	     */
	    void computeDynamic(double t, double *rpar = 0, int *ipar = 0);

	    /**
	     @brief Function to obtain the number of function evaluations
	     @return Number of function evaluations
	     */
	    virtual int functionEvalCount() = 0;

	    /**
	     @brief Function to obtain the number of computed steps of the solver
	     @return Number of computed steps
	     */
	    virtual int computedSteps() = 0;

	    /**
	     @brief Function to obtain the number of accepted steps of the solver
	     @return Number fo accepted steps
	     */
	    virtual int acceptedSteps() = 0;

	    /**
	     @brief Function to obtain the number of rejected steps of the solver
	     @return Number of rejected steps
	     */
	    virtual int rejectedSteps() = 0;


	protected:
	    /**
	     @brief Constructor
	     @sa reset
	     */
	    OdeSolve();

	    /**
	     @brief Function to abstract the allocation of memory which are required by object classes derived from this class
	     */
	    virtual void assertMemory() = 0;

	    /**
	     @brief Function to check the configuration of the object
	     */
	    virtual void assertConfig();

	    /**
	     @brief Function to check whether the current differential equation or difference equation can be solved by the mpc::OdeSolve object
	     @throw IncompatibleDGLException Is thrown if the current differential equation or difference equation cannot be solved by the mpc::OdeSolve object
	     */
	    virtual void assertValidDGL ( OdeFunction * dgl );

	    /**
	     @brief Function to abstract the computation of the solution of the differential equation or difference equation
	     @param t Terminal time for solving the differential or difference equation problem
	     @param rpar Array of double parameters
	     @param ipar Array of integer parameters
	     @throw NoInitialValueException Is thrown if no initial value has been supplied
	     @throw WrongSolverInputException Is thrown if parameters are incorrect
	     @throw TooManyStepsException Is thrown if the maximal allowed number of steps is reached
	     @throw TooSmallStepSizeException Is thrown if the current step size becomes too small
	     @throw StiffException Is thrown if the system appears to be too stiff for the solver
	     @throw yane::Utils::NotInitializedException Is thrown if the solver has not been initialized
	     */
	    virtual void abstractComputeODE(double t, double *rpar, int *ipar) = 0;

	    int _dimension;	/** @brief Dimension of the state variable of the system */

	    mpc::OdeFunction *func_;	/** @brief Pointer of the differential or difference equation */

	    mpc::OdeConfig *config_;	/** @brief Pointer of the configuration object */

	    bool initial_value_set_;	/** @brief Decision variable to check if an initial value has been assigned */

	    yane::Utils::Uuid *config_id_; 	/** @brief Pointer for configuration identifier */

	    double t_;	/** @brief Current time instance */

	    double *y_;	/** @brief Current state vector */

	    T_ODEPARAMS paramdata_;	/** @brief Structure of differential equation or difference equation data */

	    int did_;	/** @brief Error flag:
       			    - \f$ did = 1 \f$ Successful termination
			    - \f$ did = 2 \f$ Successful termination with output via yane::OdeSolve::OdeSolve::_solout
			    - \f$ did = -1 \f$ Input data is inconsistent
			    - \f$ did = -2 \f$ Maximal number of steps reached
			    - \f$ did = -3 \f$ Step size became too small
			    - \f$ did = -4 \f$ Problem seems to be too stiff */

	    double rtol_;	/** @brief Relative error tolerance */

	    double atol_;	/** @brief Absolute error tolerance */

	    double *rtol_vec_;	/** @brief Vector of relative error tolerances */

	    double *atol_vec_;	/** @brief Vector of absolute error tolerances */

	    int itol_;	/** @brief Decision variable
			    - \f$ itol = 0 \f$ Scalar error tolerances
			    - \f$ itol \geq 0 \f$ Vector error tolerances */

	    double *work_;	/** @brief Array for double variables */

	    int lwork_;	/** @brief Size of array for double variables */

	    int *iwork_;	/** @brief Array for integer variables */

	    int liwork_;	/** @brief Size of array for integer variables */


    }; //@class OdeSolve

} //@namespace odesolver

} //@namespace mpc


#endif
