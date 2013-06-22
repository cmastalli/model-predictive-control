#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

//#include <mpc/model/simulator.h>
#include <mpc/example_models/tanks_system_simulator.h>
//#include <mpc/model/model.h>

mpc::model::TanksSystemSimulator::TanksSystemSimulator()
{ 
	param1_ = 0.2552;
	param2_ = 0.0508; 
}


void mpc::model::TanksSystemSimulator::simulatePlant(double states[], double input, double samplingTime, double &output){

// Assignment of the readed variables

double V_k = input;

double Hone_k = states[0];	// Level of the first tank in the current time step
double Htwo_k = states[1];	// Level of the second tank in the current time step


// Solve the difference equations recursively

Hone_k = Hone_k + samplingTime*param1_*V_k - samplingTime*param2_*sqrt(Hone_k);

Htwo_k = Htwo_k + samplingTime*param2_*sqrt(Hone_k) - samplingTime*param2_*sqrt(Htwo_k);

output = Htwo_k;



}
