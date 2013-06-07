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
			
			/*
			@brief Function that provides the model A matrix for the LTI Tank system.
			(Linearized around the operation point: H1 = 10 cm and H2 = 10 cm)
		 	@param current_time 		Time instant
		 	@param &A 				Reference to the A matrix
			*/
			virtual void getModelParameterA(Eigen::MatrixXd& A); /*int current_time,*/

			/*
			@brief Function that provides the model B matrix for the LTI Tank system.
			(Linearized around the operation point: H1 = 10 cm and H2 = 10 cm)
		 	@param current_time 		Time instant
		 	@param &B 				Reference to the B matrix
			*/
			virtual void getModelParameterB(Eigen::MatrixXd& B); /*int current_time,*/

			/*
			@brief Function that provides the model C matrix for the LTI Tank system.
			(Linearized around the operation point: H1 = 10 cm and H2 = 10 cm)
		 	@param current_time 		Time instant
		 	@param &C 				Reference to the C matrix (already transposed, the function gives the row vector)
			*/
			virtual void getModelParameterC(Eigen::MatrixXd& C); /*int current_time,*/

			private:
			ros::NodeHandle n_;
			/* 
				Matrices of the state space representation
				Discrete state space model obtained with a sampling time of Ts=0.01s from MATLAB
			*/		

			// A matrix
			Eigen::MatrixXd Ass_;

			// B and C vector
			// The real Css matrix is the transpose of this vector, the function getModelParameterC does this.
			Eigen::MatrixXd Bss_; 

			Eigen::MatrixXd Css_;

		}; // @class Tanksystem
	
	} // @namespace test_models

} // @namespace mpc

#endif
		

		
