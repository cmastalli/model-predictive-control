#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

#include <mpc/model/simulator.h>
#include <mpc/model/TanksystemSim.h>
#include <mpc/model/model.h>

mpc::model::TanksystemSim::TanksystemSim(mpc::model::Model *model_ptr)
{
	model_ = model_ptr; 
}


void simulatePlant(double states[], double input, double samplingTime, double output){

// Assignment of the readed variables

double V_k = input;

double Hone_k = states[0];	// Level of the first tank in the current time step
double Htwo_k = states[1];	// Level of the second tank in the current time step

// Parameter creation 

double param1 = 0.2552;
double param2 = 0.0508;

// Solve the difference equations recursively

double Hone_kk;	// Level of the first tank in the k+1 time step
double Htwo_kk; // Level of the second tank in the k+1 time step

Hone_kk = Hone_k + samplingTime*param1*V_k - samplingTime*param2*sqrt(Hone_k);

Htwo_kk = Htwo_k + samplingTime*param2*sqrt(Hone_k) - samplingTime*param2*sqrt(Htwo_k);

output = Htwo_kk;

}
