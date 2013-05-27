#ifndef MPC_MODEL_TANKSYSTEMSIM_H
#define MPC_MODEL_TANKSYSTEMSIM_H


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
	class TanksystemSim : public mpc::model::Simulator
	{
	    public:
		/**
		 @brief Constructor
		 @param mpc::model::Model *model pointer to process model class
		 */
		virtual TanksystemSim(mpc::model::Model *model);

		/**
		 @brief Destructor
		 */
		virtual ~TanksystemSim();


		/**
		 @brief Function used to simulate the specified plant 
		 @param mpc::model::Model *model 	pointer to the process model class "Model"
		 */
		virtual void simulatePlant(mpc::model::Model *model);


	    protected:


	    private:

	  
	};  //@class TanksystemSim

    }  //@namespace model

}  //@namespace mpc


#endif
