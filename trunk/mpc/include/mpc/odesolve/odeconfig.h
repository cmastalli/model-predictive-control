#ifndef MPC_ODESOLVER_ODECONFIG_H
#define MPC_ODESOLVER_ODECONFIG_H

#include "typedefs.h"

namespace mpc
{

namespace odesolver
{
    /**
     This class can be used to access all the configuration possibilities of the differential equation solvers. On construction of an object of this class the configuration data is set to default values which are reasonable for standard usage. These values, however, should be adapted to the model to achieve efficiency and performance.
     @brief Class of configuration possibilities for the differential equation solvers
     */
    class OdeConfig
    {
	public:
	    /**
	     @brief Constructor setting all data to default values
	     */
	    OdeConfig();

	    /**
	     @brief Destructor
	     */
	    virtual ~OdeConfig();


	    /**
	     @brief Function to set scalar error tolerances for the adaptive step size control algorithms
	     @param rtol Relative tolerance
	     @param atol Absolute tolerance
	     @sa getTolerance
	     @throw WrongSolverInputException Is thrown if unapplicable data values are supplied
	     */
	    void setTolerance(double rtol = 1E-6, double atol = 1E-6);

	    /**
	     @brief Function to obtain the error tolerances
	     @param rtol Relative tolerance
	     @param atol Absolute tolerance
	     @sa setTolerance
	     */
	    void getTolerance(double &rtol, double &atol);

	    /**
	     @brief Function to set the vectors of error tolerances for the adaptive step size control algorithms
	     @param rtol Pointer of the relative error tolerances
	     @param atol Pointer of the absolute error tolerances
	     @throw WrongSolverInputException Is thrown if unapplicable data values are supplied
	     @sa getToleranceVector
	     */
	    void setToleranceVector(double *rtol = 0, double *atol = 0);

	    /**
	     @brief Function to obtain the vectors of currently used error tolerances
	     @param rtol Pointer of the relative error tolerance vector
	     @param atol Pointer of the absolute error tolerance vector
	     @sa setToleranceVector
	     */
	    void getToleranceVector(double * &rtol, double * &atol);

	    /**
	     @brief Function to set the maximal number of allowed integration steps
	     @param steps Maximal number of computation steps of the differential equation solver
	     @throw WrongSolverInputException Is thrown if unapplicable data values are supplied
	     @sa getMaxSteps
	     */
	    void setMaxSteps(int steps = 10000000);

	    /**
	     @brief Function to obtain the current maximal number of steps allowed for the step size control methods
	     @param steps Maximal number of computation steps of the differential equation solver
	     @sa setMaxSteps
	     */
	    void getMaxSteps(int &steps);

	    /**
	     @brief Function to set the bounds for the allowed change in the step size
		\f[ \mbox{fac}_1 \leq \frac{h_{new}}{h_{old}} \leq \mbox{fac}_2 \f]
	     @param fac1 Lower bound
	     @param fac2 Upper bound
	     @throw WrongSolverInputException Is thrown if unapplicable data values are supplied
	     @sa getStepsizeSelectionParameters
	     */
	    void setStepsizeSelectionParameters(double fac1 = 0.2, double fac2 = 10.0);

	    /**
	     @brief Function to obtain the bounds for the allowed change in the step size
	     @param fac1 Lower bound
	     @param fac2 Upper bound
	     @sa setStepsizeSelectionParameters
	     */
	    void getStepsizeSelectionParameters(double &fac1, double &fac2);

	    /**
	     @brief Function to set the maximal step size length
	     @param stepsize Maximal step size length
	     @throw WrongSolverInputException Is thrown if unapplicable data values are supplied
	     @sa getMaxStepsize
	     */
	    void setMaxStepsize(double stepsize = 0.0);

	    /**
	     @brief Function to obtain the maximal step size length
	     @param stepsize Maximal step size length
	     @sa setMaxStepsize
	     */
	    void getMaxStepsize(double &stepsize);

	    /**
	     @brief Function to set the initial step size
	     @param hinit Initial step size
	     @throw WrongSolverInputException Is thrown if unapplicable data values are supplied
	     @sa getInitialStepsize
	     */
	    void setInitialStepsize(double hinit = 0.0);

	    /**
	     @brief Function to obtain the initial step size of the used solution method for differential equations
	     @param hinit Initial step size
	     @sa setInitialStepsize
	     */
	    void getInitialStepsize(double &hinit);

	    /**
	     @brief Function to set the safety factor used within the step size control method
	     @param safetyfactor Safety factor
	     @sa getSafetyFactor
	     @throw WrongSolverInputException Is thrown if unapplicable data values are supplied
	     */
	    void setSafetyFactor(double safetyfactor = 0.9);

	    /**
	     @brief Function to obtain the safety factor used within the step size control method
	     @param safetyfactor Safety factor
	     @sa setSafetyFactor
	     */
	    void getSafetyFactor(double &safetyfactor);

	    /**
	     @brief Function to obtain the ID of the current configuration
	     @return ID of the current configuration
	     */
	    yane::Utils::Uuid *configID();

	    /**
	     @brief Static function to check validity of the supplied configuration object
	     @param config Configuration object to be checked
	    */
	    static void assertValidConfigObject(OdeConfig *config);

	    /**
	     @brief Function to copy default data to a supplied configuration object
	     @param target Configuration object which shall be configured
	     */
	    void copyTo(OdeConfig *target);

	    /**
	     @brief Function to clone the current yane::OdeSolve::OdeConfig object
	     @return Cloned yane::OdeSolve::OdeConfig object
	     */
	    virtual OdeConfig *clone();


	protected:
	    /**
	     @brief Function to change the ID of the configuration object
	     */
	    void invalidateOldConfig();

	    /**
	     @brief Function to copy all internal configuration data to the supplied configuration object
	     @param target Configuration object to hold data of calling yane::OdeSolve::OdeConfig object
	     */
	    virtual void copyToNoInvalidation(OdeConfig *target);

	    double rtol_;	/** @brief Relative tolerance */

	    double atol_;	/** @brief Absolute Tolerance */

	    double* rtol_vec_;	/** @brief Vector of relative Tolerances */

	    double* atol_vec_;	/** @brief Vector of absolute Tolerances */

	    int max_steps_;	/** @brief Maximal number of steps of the iterative method */

	    double ss_fac1_;	/** @brief Lower bound for the change of the step size */

	    double ss_fac2_;	/** @brief Upper bound for the change of the step size */

	    double max_stepsize_;	/** @brief Maximal step size */

	    double initial_stepsize_;	/** @brief Initial step size */

	    double safety_factor_;	/** @brief Safty factor */

//    std::set<std::string> config_type_id; /** Schluesselmenge, speichert die ID's der unterstuetzten Konfigurationstypen zur Identifizierung */
	    yane::Utils::Uuid *_setting_uuid; /** @brief ID of the yane::OdeSolve::OdeConfig object */


	}; //@class OdeConfig

} //@namespace odesolver


} //@namespace mpc

#endif
