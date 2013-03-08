#ifndef OPENMPC_MODEL_SIMULATOR_H
#define OPENMPC_MODEL_SIMULATOR_H

#include "../odesolve/typedefs.h"

namespace openmpc
{
    namespace model
    {
	/**< @Class Simulator
	This class provides methods to simulate a given model
	\f[ \dot{x}_{u}(t) = f(x_{u}(t, x_0), u(t, x_0)) \f]
	on a fixed time interval \f$ [t_0, T] \f$ with initial value \f$ x_{u}(t_0, x_0) = x_0 \f$ and given control \f$ u(\cdot, x_0) \f$. That is, for a given class yane::Model::Model object and a given control \f$ u \f$ the simulator can solve the differential or difference equation forward in time.
	@brief This class provides methods to simulate a given model of a process defined by a class yane::Model::Model object
	 */
	class Simulator
	{
	    public:
		/**
		 @brief Constructor
		 @param model Pointer to a class yane::Model::Model object of the model under consideration
		 @param odeconf Pointer to a class yane::OdeSolve::OdeConfig object of the model
		 @throw UndefinedException Is thrown if mass matrix, Jacobian or gradient with respect to time of the control system is not defined
		 @throw yane::Utils::MemoryException Is thrown if memory allocation failed
		 */
		Simulator(Model *model, openmpc::OdeSolve::OdeConfig *odeconf = 0);

		/**
		 @brief Destructor
		 */
		virtual ~Simulator();

		/**
		 This function initializes the class yane::Model::Simulator object.
		 @param t Time instant
		 @param x State vector
		 @param u Control vector
		 @brief Function to initialize the class yane::Model::Simulator object
		 */
		virtual void initialize(double t, double *x, double *u);

		/**
		 This function sets the solution output function.
		 @param solout
		 @brief Function to set solution output function
		 */
		virtual void setControl(double *u);

		/**
		 This function sets the solution output function.
		 @param solout
		 @brief Function to set solution output function
		 */
		virtual void setSolutionOutput(OdeSolve::T_SOLOUTFUNC func);

		/**
		 This function triggers the simulator to solve the dynamics of the control system under consideration up to a time instant defined in the call of this function.
		 @param targettime Endpoint of the simulation
		 @brief Function to start the simulator
		 */
		virtual void run(double targettime);

		/**
		 This function triggers the simulator to solve the dynamics of the control system under consideration on a given sampling grid and control.
		 @param seqlength Time index of endpoint on time grid
		 @param t Time grid
		 @param x State vector
		 @param u Control on the time grid
		 @brief Function to trigger the simulator to solve the dynamics of the control system under consideration on a given sampling grid and control
		 */
		virtual void predictSequence(int seqlength, double *t, double *x, double *u);

		/**
		 This function triggers the simulator to compute the state trajectory of the control system under consideration on a given sampling grid and control.
		 @param seqlength Length of the computed state trajectory
		 @param t Time grid
		 @param x State vector
		 @param u Control on the time grid
		 @brief Function to trigger the simulator to compute the state trajectory of the control system under consideration on a given sampling grid and control
		 */
		virtual void trajectory( int seqlength, double *t, double *x, double *u, double *trajectory);

	    protected:
		/**
		 @brief Static differential or difference equation which has been transformed from a control system to a parametrized system
		 @param n Dimension of the state
		 @param t Time grid
		 @param y State vector
		 @param dy Derivative of the state
		 @param rpar Real valued parameters
		 @param ipar Integer valued parameters
		 */
		static void staticOdeFunc(int *n, double *t, double *y, double *dy, double *rpar, int *ipar);

		/**
		 @brief Static function of the Jacobian of the constraints
		 @param n Dimension of the state
		 @param t Time grid
		 @param y State vector
		 @param dfy Jacobian of the constraints
		 @param ldfy Dimension of the Jacobian
		 @param rpar Real valued parameters
		 @param ipar Integer valued parameters
		 */
		static void staticJacFunc(int *n, double *t, double *y, double *dfy, int *ldfy, double *rpar, int *ipar);

		/**
		 @brief Static function of the mass matrix of the system
		 @param n Dimension of the state
		 @param mas Mass matrix of the system
		 @param lmas Dimension of the mass matrix
		 @param rpar Real valued parameters
		 @param ipar Integer valued parameters
		 */
		static void staticInertialFunc(int *n, double *mas, int *lmas, double *rpar, int *ipar);

		/**
		 @brief Static function of the time derivative of the control system
		 @param n Dimension of the state
		 @param t Time grid
		 @param y State vector
		 @param dft Time derivative of the system
		 @param rpar Real valued parameters
		 @param ipar Integer valued parameters
		 */
		static void staticDftFunc(int *n, double *t, double *y, double *dft, double *rpar, int *ipar);


		Model *model_;     /**< @brief Pointer of class yane::Model::Model object */
		openmpc::OdeSolve::OdeSolveFirst *odesolver_; /**< @brief Pointer of class yane::OdeSolve::OdeSolveFirst object */
		openmpc::OdeSolve::OdeFunction *odefunc_;  /**< @brief Pointer of class yane::OdeSolve::OdeFunction object */
		openmpc::OdeSolve::OdeConfig *odeconfig_;  /**< @brief Pointer of class yane::OdeSolve::OdeConfig object */

		double *x_;     /**< @brief State variable */
		double *u_;     /**< @brief Control variable */

		int dimension_x_;    /**< @brief Dimension of the state */
		int dimension_u_;    /**< @brief Dimension of the control */

	};  //@class Simultor

    }  //@namespace model

}  //@namespace openmpc

#endif  //OPENMPC_MODEL_SIMULATOR_H
