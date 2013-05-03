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
			@brief Function that provides the model matrices for the LTI Tank system.
			(Linearized around the operation point: H1 = 10 cm and H2 = 10 cm)
		 	@param current_time 		Time instant
		 	@param &A 				Reference to the A matrix
		 	@param &B 				Reference to the B matrix
		 	@param &C 				Reference to the C matrix 
			*/
			virtual void getModelParameters(int current_time, MatrixXd& A, MatrixXd& B, MatrixXd& C);

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
			// The real Css matrix is the transpose of this vector so when using it, it must be used as Css.transpose()
			Vector2d Css<< 0.0000,
			1.0000;

		} // Class Tanksystem
	
	} // Namespace test_models

} // Namespace mpc


		

		
