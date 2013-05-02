#ifndef TANKSYSTEM_H
#define TANKSYSTEM_H

#include "model/model.h"
#include <Eigen/Dense>

namespace mpc
{
	namespace test_models
	{

		class tanksystem : public mpc::model::model
		{

		public:

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

		private:

		} // Class tanksystem
	
	} // Namespace test_models

} // Namespace mpc


		

		
