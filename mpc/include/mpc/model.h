#ifndef MPC_MODEL_H
#define MPC_MODEL_H


namespace mpc
{
    /**
     @class Model
     @brief Abstract class to define the model of the process and the optimal control problem to be solved
     */
    class Model
    {
	public:
	    virtual ~Model();

	    /**
	     @brief Function to set the dimension of the state
	     @param dimension New dimension of the state
	     */
	    virtual void setDimensionState(int dimension);

	    /**
	     @brief Function to obtain the dimension of the state
	     @return Dimension of the state
	     */
	    virtual int dimensionState();

	    /**
	    @brief Function to set the dimension of the control
	    @param crtl_dimension New dimension of the control
	     */
	     virtual void setDimensionControl(int ctrl_dimension);

	    /**
	     @brief Function to obtain the dimension of the control
	     @return Dimension of the control
	     */
	    virtual int getDimensionControl();

	    /**
	     @brief Function to set the number of restrictions
	     @return Number of restrictions
	     */
	    virtual int setRestrictionSize(int restr_size);

	    /**
	     @brief Function to obtain the number of restrictions
	     @return Number of restrictions
	     */
	    virtual int getRestrictionSize();

	    /**
	     @brief Function to obtain the number of double parameters
	     @return Number of double parameters
	     */
	    virtual int getDoubleParameterSize();

	    /**
	     @brief Function to obtain the number of integer parameters
	     @return Number of inteter parameters
	     */
	    virtual int getIntParameterSize();

	    /**
	     @brief Function to set the value of a double parameter
	     @param param_id Index of the parameter
	     @param value Value of the parameter
	     */
	     virtual void setDoubleParameter(int param_id, double value);

	    /**
	     @brief Function to set the value of a integer parameter
	     @param param_id Index of the parameter
	     @param value Value of the parameter
	     */
	    virtual void setIntParameter(int param_id, int value);

	    /**
	     @brief Function to obtain the value of a double parameter
	     @param param_id Index of the parameter
	     @return Value of the parameter
	     */
	    virtual double setDoubleParameter(int param_id);

	    /**
	     @brief Function to obtain the value of a integer parameter
	     @param param_id Index of the parameter
	     @return Value of the parameter
	     */
	    virtual int getIntParameter(int param_id);

	    /**
	     @brief Function to obtain the pointer of the mpc::OdeSolve::OdeSolve object which is suitable for the considered class mpc::Model object
	     @return Pointer of the yane::OdeSolve::OdeSolve object
	     */
	    virtual mpc::OdeSolve::OdeSolveFirst *odeSolver();

	    /**
	     @brief Function to obtain the pointer of the mpc::OdeSolve::OdeConfig object which is suitable for the mpc::OdeSolve::OdeSolve object defined in the considered class mpc::Model object
	     @return Pointer of the tmb::OdeSolve::OdeConfig object
	     */
	    virtual mpc::OdeSolve::OdeConfig *odeConfig();

	    /**
	     @brief Function to implement the dynamic of the system
		\f[ \dot{x}_{u}(t) = f(t, x_{u}(t, x_0), u(t, x_0)) \f]
	     @param t Time instant
	     @param x State vector
	     @param u Control vector
	     @param dx Values of the derivatives
	     */
	    virtual void odeFunction(double t, double *x, double *u, double *dx) = 0;

	    /**
	     @brief Function to implement the integral part of the cost function
		\f[ L \left( t, x_{u}(t, x_0), u(t, x_0) \right) \f]
	     @param t Time instant
	     @param x State vector
	     @param u Control vector
	     @return Value of the integral function
	     */
	    virtual double continuousCostfunction(double t, double *x, double *u) = 0;

	    /**
	     @brief Function to implement the nonlinear constraints
		\f{eqnarray*}{
		x_{u}(t, x_0) & \in & X \qquad \forall t \in [t_0, t_N] \\
		u(t, x_0) & \in & U \qquad \forall t \in [t_k, t_k)
		\f}
	     @param t Time instant
	     @param x State vector
	     @param u Control vector
	     @param fx Values of the constraints
	     @sa getRestrictionSize
	     */
	    virtual void restrictionFunction(double t, double *x, double *u, double *fx) = 0;

	    /**
	     @brief Function to implement the mass matrix of the control system
	     @param mass Mass matrix
	     */
	    virtual void massMatrix(double *mass);

	    /**
	     @brief Function to implement the Jacobian of the constraints of the control system
	     @param t Time instant
	     @param x State vector
	     @param u Control vector
	     @param dy Values of the Jacobian
	     */
	    virtual void jacobiMatrix(double t, double *x, double *u, double *dy);

	    /**
	     @brief Function to implement the gradient of the cost function of the control system
	     @param t Time instant
	     @param x State vector
	     @param u Control vector
	     @param grad Gradient of the cost function of the control system
	     */
	    virtual void costfunctionGradient(double t, double *x, double *u, double *grad);

	    /**
	     @brief Function to implement the derivative with respect to time of the dynamic of the control system
	     @param t Time instant
	     @param x State vector
	     @param u Control vector
	     @param dft Time derivative of the dynamic of the control system
	     */
	     virtual void dynamicDerivativeT(double t, double *x, double *u, double *dft);

	    /**
	     @brief Function to implement the derivative with respect to time of the cost function of the control system
	     @param t Time instant
	     @param x State vector
	     @param u Control vector
	     @param dot Time derivative of the cost function of the control system
	     */
	    virtual void costfunctionDerivativeT(double t, double *x, double *u, double *dot);

	    /**
	     @brief Function to implement the discrete running cost as well as the terminal cost term of the cost function of the control system
		\f[ l \left( t_k, x_{u}(t_k, x_0), u(t_k, x_0) \right) \qquad \mbox{and} \qquad F(t_N, x_{u}(t_N, x_0)) \f]
	     @param length Length of the sum
	     @param horizont Length of the optimization horizon
	     @param t Time grid
	     @param x Open loop state trajectory on the time grid
	     @param u Open loop control trajectory on the time grid
	     @return Value of the discrete running cost
	     */
	    virtual double discreteCostfunction(int length, int horizon, double *t, double *x, double * u) = 0;

	    /**
	     @brief Function to put different weights on the integral and discrete time parts of the cost functional
	     @param obj_weight Weighting factor of the integral part of the cost function
	     @param pointcost_weight Weighting factor of the discrete time part of the cost function
	     */
	    virtual void getObjectiveWeight(double &obj_weight, double &pointcost_weight) = 0;

	    /**
	     @brief Function to obtain the box constraints of the control
	     @param lb Pointer on lower bounds of the control
	     @param ub Pointer on upper bounds of the control
	     @sa getDimensionControl
	     */
	    virtual void getControlBounds(double *lb, double *ub);

	    /**
	     @brief Function to set the box constraints of the control
	     @param lb Pointer on lower bounds of the control
	     @param ub Pointer on upper bounds of the control
	     */
	    virtual void setControlBounds(double *lb, double *ub);

	    /**
	     @brief Function to get the box constraints of the state
	     @param lb Pointer on lower bounds of the state
	     @param ub Pointer on upper bounds of the state
	     @sa getDimension
	     */
	    virtual void getModelBounds(double *lb, double *ub);

	    /**
	     @brief Function to set the box constraints of the state
	     @param lb Pointer on lower bounds of the state
	     @param ub Pointer on upper bounds of the state
	     */
	    virtual void setModelBounds(double *lb, double *ub);

	    /**
	     @brief Function to get the default values defined for the control system under consideration
	     @param x Pointer for the default state
	     */
	     virtual void getDefaultState(double *x) = 0;

	    /**
	     @brief Function to get the default values of the control defined for the control system under consideration
	     @param u Pointer for the default control
	     */
	    virtual void getDefaultControl(double *u) = 0;

	    /**
	     @brief Function to implement the internal resets for the mpc::ModelPredictiveControl class object at each iterate of the MPC process
	     @param horizon Optimization horizon
	     @param timesteps Time grid
	     @param x State vector
	     @param sdatavalues Shooting node values
	     */
	    virtual void eventBeforeMPC(int horizon, double *timesteps, double *x, double *sdatavalues);

	    virtual void exactDiscreteCostfunctionDerivativeX(int length, int horizont, double *t, double *x, double *u, double *fx);

	    virtual void exactContinuousCostfunctionDerivativeX(double t, double *x, double *u, double *fx);

	    virtual void exactContinuousCostfunctionDerivativeU(double t, double *x, double *u, double *fx);

	    virtual void exactOdeFunctionDerivativeX(double t, double *x, double *u, double **fx);

	    virtual void exactOdeFunctionDerivativeU(double t, double *x, double *u, double **fx);

	    virtual void exactRestrictionFunctionDerivativeX(double t, double *x, double *u, double **fx);

	    virtual void exactRestrictionFunctionDerivativeU(double t, double *x, double *u, double **fx);

	    //virtual const yane::Utils::Uuid & id();

	    virtual const char *name();

	    virtual const char *description();

	    virtual const char *stateName(int dimindex);

	    virtual const char *controlName(int dimindex);

	    virtual const char *doubleParameterName(int parindex);

	    virtual const char *intParameterName(int parindex);
				
	    /**
	     @brief Function to implement the nonlinear constraints of the decentralized control system with are induced by neighbouring systems
	     @param t Time instant
	     @param x State vector
	     @param u Control vector
	     @param x_neighbour State vector of the neighbouring system
	     @param u_neighbour Control vector of the neighbouring system
	     @param fx Values of the constraints
	     @sa getRestrictionSize
	     */
	    virtual void networkRestrictionFunction(double t, double *x, double *u, double *x_neighbour, double *u_neighbour, double *fx);

	    /**
	     @brief Function to set the number of restrictions due to the network context
	     @return Number of restrictions due to the network context
	     */
	    virtual void setNetworkRestrictionSize(int network_restr_size);

	    /**
	     @brief Function to obtain the number of restrictions due to the network context
	     @return Number of restrictions due to the network context
	     */
	    virtual int networkRestrictionSize();

	    /**
	     @brief Function to implement the discrete running cost as well as the terminal cost term of the cost function of the control system in a networked context
	     @param length Length of the sum
	     @param horizon Length of the optimization horizon
	     @param t Time grid
	     @param x Open loop state trajectory on the time grid
	     @param u Open loop control trajectory on the time grid
	     @param x_neighbour State vector of the neighbouring system
	     @param u_neighbour Control vector of the neighbouring system
	     @return Value of the discrete running cost
	     */
	    virtual double networkCostfunction(int length, int horizon, double *t, double *x, double *u, double *x_neighbour, double *u_neighbour, double *multiplier);


	protected:
	    /**
	     @brief Constructor
	     @param ode Pointer to the used yane::OdeSolve::OdeSolveFirst object
	     @param odeconf Pointer to the used yane::OdeSolve::OdeConfig object
	     @param dimension Dimension of the state variable
	     @param ctrl_dimension Dimension of the control variable
	     @param restrsize Number of nonlinear constraints
	     @param paramsize Number of double parameters
	     @param iparamsize Number of integer parameters
	     @param networkrestrsize Number of restrictions per model arising from the network context
	     */
	    Model(mpc::OdeSolve::OdeSolveFirst *ode, mpc::OdeSolve::OdeConfig *odeconf, int dimension, int ctrl_dimension, int restr_size, int param_size = 0, int iparamsize = 0, int networkrestrsize = 0);


	    int dimension_;    /** @brief Dimension of the state variable */

	    int ctrl_dimension_;   /** @brief Dimension of the control variable */

	    int paramsize_;    /** @brief Number of double parameters */

	    int iparamsize_;   /** @brief Number of integer parameters */

	    int restrsize_;    /** @brief Number of nonlinear constraints */

	    int network_restrsize_;    /** @brief Number of nonlinear constraints due to the network context */

	    double *params_;   /** @brief Array of double parameters */

	    int *iparams_;    /** @brief Array of integer parameters */

	    mpc::OdeSolve::OdeSolveFirst *ode_; /** @brief Pointer to the used mpc::OdeSolve::OdeSolveFirst object */

	    mpc::OdeSolve::OdeConfig *odeconfi_g; /** @brief Pointer to the used mpc::OdeSolve::OdeConfig object */

	    double *control_lb_;   /** @brief Array of lower bounds on the control */

	    double *control_ub_;   /** @brief Array of upper bounds on the control */

	    double *state_lb_;   /** @brief Array of lower bounds on the state */

	    double *state_ub_;   /** @brief Array of upper bounds on the state */


	private:
	    /**
	     * @brief Locked constructor
	     */
	    Model();

	    /**
	     * @brief Locked constructor
	     */
	    Model(Model &input);


    };  //@class Model


}  //@namespace mpc

}

#endif

