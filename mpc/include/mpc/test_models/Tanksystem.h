#ifndef TANKSYSTEM_H
#define TANKSYSTEM_H

#include "model.h"
#include <Eigen/Dense>

namespace mpc
{
	namespace test_models
	{

		class Tanksystem : public mpc::model::Model
		{

		public:

			// Constructor
			Tanksystem();

			// Destructor
			~Tanksystem();
			
			/*
			@brief Function that provides the model A matrix for the LTI Tank system.
			(Linearized around the operation point: H1 = 10 cm and H2 = 10 cm)
		 	@param current_time 		Time instant
		 	@param &A 				Reference to the A matrix
			*/
			virtual void getModelParameterA(int current_time, MatrixXd& A);

			/*
			@brief Function that provides the model B matrix for the LTI Tank system.
			(Linearized around the operation point: H1 = 10 cm and H2 = 10 cm)
		 	@param current_time 		Time instant
		 	@param &B 				Reference to the B matrix
			*/
			virtual void getModelParameterB(int current_time, MatrixXd& B);

			/*
			@brief Function that provides the model C matrix for the LTI Tank system.
			(Linearized around the operation point: H1 = 10 cm and H2 = 10 cm)
		 	@param current_time 		Time instant
		 	@param &C 				Reference to the C matrix (already transposed, the function gives the row vector)
			*/
			virtual void getModelParameterC(int current_time, MatrixXd& C);

		private:

			/* 
				Matrices of the state space representation
				Discrete state space model obtained with a sampling time of Ts=0.01s from MATLAB
			*/		

			// A matrix
			Matrix2d Ass<< 0.9992, 0.0000
			-0.000803, 1.001;

			// B matrix
			Vector2d Bss<< 0.002551,
			0.0000;

			// C matrix
			// The real Css matrix is the transpose of this vector, the function getModelParameterC does this.
			Vector2d Css<< 0.0000,
			1.0000;

		} // Class Tanksystem
	
	} // Namespace test_models

} // Namespace mpc


		

		
