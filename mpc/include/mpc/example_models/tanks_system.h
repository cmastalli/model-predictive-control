#ifndef TANKSSYSTEM_H
#define TANKSSYSTEM_H

#include "mpc/model/model.h"


namespace mpc
{
	namespace example_models
	{

		class TanksSystem : public mpc::model::Model
		{
			public:
				// Constructor
				TanksSystem(ros::NodeHandle node);
				
				
				// Destructor
				~TanksSystem();
				
				
				void computeLTIModel();
				
			
				virtual bool computeDynamicModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C);


			private:
				ros::NodeHandle nh_;


		}; //@class TanksSystem
	
	} //@namespace example_models

} //@namespace mpc


#endif



