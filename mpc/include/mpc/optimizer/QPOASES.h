#ifndef QPOASES_H
#define QPOASES_H

#include <mpc/optimizer/optimizer.h>
#include <ros/ros.h>

namespace mpc
{
	namespace optimizer
	{
	
		class QPOASES : public mpc::optimizer::Optimizer
		{
			public:
			// Constructor
			QPOASES(ros::NodeHandle node, mpc::model::Model *model_ptr);

			//Destructor
			~QPOASES();


		   /**
			 @brief Function to solve the optimization problem formulated in the MPC  
			 @param Eigen::VectorXd x_k 		state vector
			 @param Eigen::VectorXd x_ref		reference vector 
			 */
			virtual void computeMPC(Eigen::VectorXd x_k, Eigen::VectorXd x_ref);

			private:

			mpc::model::Model *model_;
			
			int states_, inputs_, horizon_;

			ros::NodeHandle n_;

			//int &nWSR;	//number of working set recalculations

		}; // @class QPOASES

	} // @namespace optimizer

} // @namespace mpc

#endif
