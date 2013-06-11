#ifndef TANKSYSTEM_H
#define TANKSYSTEM_H

#include "mpc/model/model.h"


namespace mpc
{
	namespace test_models
	{

		class Tanksystem : public mpc::model::Model
		{
			public:
				// Constructor
				Tanksystem(ros::NodeHandle node);
				
				
				// Destructor
				~Tanksystem();
				
				
				void computeLTIModel();
				
			
				virtual bool computeDynamicModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C);

			
			/*
			@brief Function that provides the model A matrix for the LTI Tank system.
			(Linearized around the operation point: H1 = 10 cm and H2 = 10 cm)
		 	@param current_time 		Time instant
		 	@param &A 				Reference to the A matrix
			*/
//			virtual void getModelParameterA(Eigen::MatrixXd& A); /*int current_time,*/


			private:
				ros::NodeHandle nh_;


		}; //@class Tanksystem
	
	} //@namespace test_models

} //@namespace mpc


#endif



