#ifndef MPC_MODEL_SIMULATOR_H
#define MPC_MODEL_SIMULATOR_H


namespace mpc
{

    namespace model
    {

	/**< @Class Simulator
	 This class provides methods to simulate a given model
	 \f[ \dot{x}(t) = Ax(t) + Bu(t) \f]
	 \f[ y(t) = Cx(t) \f]
	 on a fixed prediction horizon interval \f$ [t_0, t_N] \f$ with initial value \f$ x(t_0, x_0) = x_0 \f$ and given control \f$ u(\cdot, x_0) \f$. That is, for a given class mpc::model::Model object and a given control \f$ u \f$ the simulator can solve the differential or difference equation forward in time.
	 @brief This class provides methods to simulate a given model of a process defined by a class mpc::model::Model object
	 */
	class Simulator
	{
	    public:
		/**
		 @brief Constructor
		 @param mpc::model::Model *model pointer to process model class
		 */
		virtual Simulator(mpc::model::Model *model);

		/**
		 @brief Destructor
		 */
		virtual ~Simulator();

		/**
		 This function initializes the class mpc::model::Simulator object.
		 @param t Time instant
		 @param x State vector
		 @param u Control vector
		 @brief Function to initialize the class mpc::model::Simulator object
		 */
		virtual void initialize(double t, double *x, double *u);

		/**
		 @brief Function used to obtain the predictions from the process model for a time horizon
		 @param int *prediction_horizon 	pointer to the time horizon of prediction 
		 @param mpc::model::Model *model 	pointer to the process model class "Model"
		 */
		virtual void predictState(int *prediction_horizon, mpc::model::Model *model);


	    protected:


	    private:

	  
	};  //@class ModelPredictiveControl

    }  //@namespace model

}  //@namespace mpc


#endif
