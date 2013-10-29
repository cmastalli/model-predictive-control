#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <fstream>


// Global variables 

std::vector<std::vector<double> > in_;
std::vector<std::vector<double> > out_;
std::vector<double> time_;
std::string input_data_name = "/home/rene/ros_workspace/model-predictive-control/mpc/data/validation/validation_input_data.txt";
std::string output_data_name = "/home/rene/ros_workspace/model-predictive-control/mpc/data/validation/validation_output_data.txt";

void computeLTIModel(Eigen::MatrixXd &A_, Eigen::MatrixXd &B_, double* point_states_ptr, double* input_states_ptr)
{
	

	Eigen::Map<Eigen::VectorXd> x(point_states_ptr, 12);
	Eigen::Map<Eigen::VectorXd> u(input_states_ptr, 4);
	
	// A matrix
	double Ct_ = 8.17e-006; // Thrust coefficient [N/(rad/s)^2]
	double Cq_ = 2.17e-007; // Drag coefficient [Nm/(rad/s)^2]
	double Ixx_ = 2.04016e-003; // Inertia around the X axis [Kg/m^3]
	double Iyy_ = 1.5677e-003; // Inertia around the Y axis [Kg/m^3]
	double Izz_ = 3.51779e-003; // Inertia around the Z axis [Kg/m^3]
	double m_ = 0.4305; // Mass of the quadrotor [Kg]
	double d_ = 0.35; // Distance from the rotor to the mass center of the quadrotor [m]
	double Jr_ = 1.66e-005; // Inertia of a unitary rotor (approximated as a disc) [Kg*m^2] //FIXME Inertia 
	double At_ = 0.0083; // Sampling time
	

	/** For convenience, we will define Cm as follows: **/	
	double Cm = Ct_/m_;

	A_(0,3) = 1*At_;
	A_(1,4) = 1*At_;
	A_(2,5) = 1*At_;
	A_(3,6) = (sin(x(8))*cos(x(6)) - cos(x(8))*sin(x(7))*sin(x(6)))*u(0)*At_;
	A_(3,7) = (cos(x(8))*cos(x(7))*cos(x(6)))*u(0)*At_;
	A_(3,8) = (cos(x(8))*sin(x(6)) - sin(x(8))*sin(x(7))*cos(x(6)))*u(0)*At_;
	A_(4,6) = -(sin(x(8))*sin(x(7))*sin(x(6)) + cos(x(8))*cos(x(6)))*u(0)*At_;
	A_(4,7) = (sin(x(8))*cos(x(7))*cos(x(6)))*u(0)*At_;
	A_(4,8) = (cos(x(8))*sin(x(7))*cos(x(6)) + sin(x(8))*sin(x(6)))*u(0)*At_;
	A_(5,6) = -(cos(x(7))*sin(x(6)))*u(0)*At_;
	A_(5,7) = -(sin(x(7))*cos(x(6)))*u(0)*At_;
	A_(6,6) = x(10)*cos(x(6))*tan(x(7))*At_ - x(11)*sin(x(6))*tan(x(7))*At_ + 1;
	A_(6,7) = x(10)*sin(x(6))*(1/(cos(x(7))*cos(x(7))))*At_ + x(11)*cos(x(6))*(1/(cos(x(7))*cos(x(7))))*At_;
	A_(6,9) = 1*At_;
	A_(6,10) = sin(x(6))*tan(x(7))*At_;
	A_(6,11) = cos(x(6))*tan(x(7))*At_;
	A_(7,6) = -x(10)*sin(x(6))*At_ - x(11)*cos(x(6))*At_;
	A_(7,10) = cos(x(6))*At_;
	A_(7,11) = -sin(x(6))*At_;
	A_(8,6) = x(10)*cos(x(6))*(1/cos(x(7)))*At_ - x(11)*sin(x(6))*(1/cos(x(7)))*At_;
	A_(8,7) = x(10)*sin(x(6))*(1/cos(x(7)))*tan(x(7))*At_ + x(11)*cos(x(6))*(1/cos(x(7)))*tan(x(7))*At_;
	A_(8,10) = (sin(x(6))/cos(x(7)))*At_;
	A_(8,11) = (cos(x(6))/cos(x(7)))*At_;
	A_(9,10) = (Iyy_ - Izz_)*(x(11)/Ixx_)*At_;
	A_(9,11) = (Iyy_ - Izz_)*(x(10)/Ixx_)*At_;
	A_(10,9) = (Izz_ - Ixx_)*(x(11)/Iyy_)*At_;
	A_(10,11) = (Izz_ - Ixx_)*(x(9)/Iyy_)*At_;
	A_(11,9) = (Ixx_ - Iyy_)*(x(10)/Izz_)*At_;
	A_(11,10) = (Ixx_ - Iyy_)*(x(9)/Izz_)*At_;

	//std::cout <<"The A matrix for the desired linear operation point is:\n" << A_ << std::endl;
	
	B_(3,0) = (cos(x(8))*sin(x(7))*cos(x(6))*At_ + sin(x(8))*sin(x(6)))*At_;
	//B_(3,1) = (cos(x(8))*sin(x(7))*cos(x(6))*At_ + sin(x(8))*sin(x(6)))*At_;
	//B_(3,2) = (cos(x(8))*sin(x(7))*cos(x(6))*At_ + sin(x(8))*sin(x(6)))*At_;
	//B_(3,3) = (cos(x(8))*sin(x(7))*cos(x(6))*At_ + sin(x(8))*sin(x(6)))*At_;		

	B_(4,0) = (sin(x(8))*sin(x(7))*cos(x(6))*At_ - cos(x(8))*sin(x(6)))*At_;
	//B_(4,1) = (sin(x(8))*sin(x(7))*cos(x(6))*At_ - cos(x(8))*sin(x(6)))*At_;
	//B_(4,2) = (sin(x(8))*sin(x(7))*cos(x(6))*At_ - cos(x(8))*sin(x(6)))*At_;
	//B_(4,3) = (sin(x(8))*sin(x(7))*cos(x(6))*At_ - cos(x(8))*sin(x(6)))*At_;
	
	B_(5,0) = (cos(x(8))*cos(x(6)))*At_;
	//B_(5,1) = (cos(x(8))*cos(x(6)))*At_;
	//B_(5,2) = (cos(x(8))*cos(x(6)))*At_;
	//B_(5,3) = (cos(x(8))*cos(x(6)))*At_;

	B_(9,1) = At_/Ixx_;

	B_(10,2) = At_/Iyy_;

	B_(11,3) = (1/Izz_)*At_;

	
	//std::cout <<"The B matrix for the desired linear operation point is:\n" << B_ << std::endl;
	
}

bool writeToDisc()
{
	bool data = true;
	if (in_[0].size() == 0) {
		ROS_WARN("Could not save the data because the input signal information is NULL");
		data = false;
	}
	if (out_[0].size() == 0) {
		ROS_WARN("Could not save the data because the output signal information is NULL");
		data = false;
	}

	
	if (data) {
		
		
		std::ofstream infile;
		infile.open(input_data_name.c_str());
		infile.precision(4);
		infile.setf(std::ios::fixed, std::ios::floatfield);
		infile.setf(std::ios::left, std::ios::adjustfield);

		infile << "t\t" << "u1\t" << "u2\t" << "u3\t" << "u4\t" << std::endl;
		
		for (unsigned int j = 0; j < in_[0].size(); j++) {
			infile << time_[j];
			for (unsigned int i = 0; i < in_.size(); i++) {
				infile << '\t';
				infile << in_[i][j];
			}
			infile	<< std::endl;
		}
		infile.close();
		ROS_INFO("Input data validation file recorded.");

		std::ofstream outfile;
		outfile.open(output_data_name.c_str());
		outfile.precision(4);
		outfile.setf(std::ios::fixed, std::ios::floatfield);
		outfile.setf(std::ios::left, std::ios::adjustfield);

		outfile << "t\t" << "x\t" << "y\t" << "z\t" << "u\t" << "v\t" << "w\t" << "roll\t" << "pitch\t" << "yaw\t" << "p\t" << "q\t" << "r" << std::endl;


    	for (unsigned int j = 0; j < out_[0].size(); j++) {
			outfile << time_[j];
			for (unsigned int i = 0; i < out_.size(); i++) {
				outfile << '\t';
				outfile << out_[i][j];
			}
			outfile	<< std::endl;
		}
		outfile.close();
		ROS_INFO("Output data validation file recorded.");
	}

	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "model_validation");

	Eigen::VectorXd delta_outputs(12);

	Eigen::VectorXd delta_states(12); //= Eigen::VectorXd::Zero(12);
	delta_states(0) = 0.0;
	delta_states(1) = 0.0;
	delta_states(2) = 0.0;
	delta_states(3) = 0.0;
	delta_states(4) = 0.0;
	delta_states(5) = 0.0;
	delta_states(6) = 0.0;
	delta_states(7) = 0.0;
	delta_states(8) = 0.0;
	delta_states(9) = 0.0;
	delta_states(10) = 0.0;
	delta_states(11) = 0.0;
	Eigen::MatrixXd A_ = Eigen::MatrixXd::Identity(12, 12);	
	Eigen::MatrixXd B_ = Eigen::MatrixXd::Zero(12, 4);



	/** Desired operation point **/
	double state_point1[12] = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double input_point1[4] = {0.4305*9.81, 0.0, 0.0, 0.0};

	computeLTIModel(A_, B_, state_point1, input_point1);
	double current_time;

	in_.resize(4);
	out_.resize(12);
	time_.push_back(0.0);
	for(int j=0; j<4; j++){
		in_[j].push_back(0.0);
	}
	for (int j=0; j<12; j++){
		out_[j].push_back(0.0);
 	}

	
	/** Input **/
	for (unsigned long int i=0; i<10000; i++){ 

		//ROS_INFO("Hola!");
		current_time = i*0.0083;	


	// A movement in the Y axis is changing the rotational speed of the (1,3) pair of rotors
	Eigen::VectorXd deltaInput(4);
	//double w1 = 400.0;	//*(1 - 0.1*sin(300.0*current_time)); 
	//double w2 = 400.0;
	//double w3 = 400.0;	//*(1 - 0.1*cos(300.0*current_time));
	//double w4 = 400.0;
	
	if (current_time > 20 && current_time < 25){ 
		deltaInput(2) = 1.0; //*(1 - exp(-0.05*current_time));
	}
	else{
		deltaInput(2) = 0.0;
	}
	deltaInput(1) = 0.0;  //*(1 - exp(-0.05*current_time));
	deltaInput(0) = 0.0; 
	deltaInput(3) = 0.0;

	//std::cout << "w1\t" << w1 << "w2\t" << w2 << "w3\t" << w3 << "w4\t" << w4 <<std::endl;
	
	delta_outputs = A_*delta_states + B_*deltaInput;

	std::cout << "output z\t" << delta_outputs(2) << "\tvs output out[2]\t" << out_[2][i] << std::endl;

	in_[0].push_back(deltaInput(0) + input_point1[0]);
	in_[1].push_back(deltaInput(1) + input_point1[1]);
	in_[2].push_back(deltaInput(2) + input_point1[2]);
	in_[3].push_back(deltaInput(3) + input_point1[3]);

	out_[0].push_back(delta_outputs(0) + state_point1[0]);
	out_[1].push_back(delta_outputs(1) + state_point1[1]);
	out_[2].push_back(delta_outputs(2) + state_point1[2]);
	out_[3].push_back(delta_outputs(3) + state_point1[3]);
	out_[4].push_back(delta_outputs(4) + state_point1[4]);
	out_[5].push_back(delta_outputs(5) + state_point1[5]);
	out_[6].push_back(delta_outputs(6) + state_point1[6]);
	out_[7].push_back(delta_outputs(7) + state_point1[7]);
	out_[8].push_back(delta_outputs(8) + state_point1[8]);
	out_[9].push_back(delta_outputs(9) + state_point1[9]);
	out_[10].push_back(delta_outputs(10) + state_point1[10]);
	out_[11].push_back(delta_outputs(11) + state_point1[11]);

	time_.push_back(current_time);
	//std::cout << "x\t" << out_[0][i] << "\ty\t" << out_[1][i] << "\tz\t" << out_[2][i] << "\tu\t" << out_[3][i] << "\tv\t" << out_[4][i] << "\tw\t" << out_[5][i] << "\troll\t" << out_[6][i] << "\tpitch\t" << out_[7][i] << "\tyaw\t" << out_[8][i] << "\tp\t" << out_[9][i] << "\tq\t" << out_[10][i] << "\tr\t" << out_[11][i] << std::endl;
	//std::cout << "time\t" << time_[i] << "\tvs counter\t" << i << std::endl; 
	delta_states = delta_outputs;
	i++;

	}

	/**for (int j=0; j<in_[0].size(); j++){
		std::cout <<"Input vector" << in_[0][j] << std::endl;
	}**/

	bool write = false;
	write = writeToDisc();
	if (write){
		std::cout << "Satisfactory recorded validation data." << std::endl;
	}

	std::cout << "A matrix" << std::endl << A_ << "B matrix" << std::endl << B_ << std::endl;

	return 0;


}
