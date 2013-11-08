#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <fstream>


// Global variables 
std::vector<std::vector<double> > u_;
std::vector<std::vector<double> > x_;
std::vector<std::vector<double> > x_NL_;
std::vector<double> time_;
std::string input_data_name = "ros_workspace/model-predictive-control/mpc/data/validation/validation_input_data.txt";
std::string output_data_name = "ros_workspace/model-predictive-control/mpc/data/validation/validation_output_data.txt";
std::string nonlinear_data_name = "ros_workspace/model-predictive-control/mpc/data/validation/validation_nonlinear_data.txt";

// Parameters of the dynamic model
double Ct_ = 8.17e-006; // Thrust coefficient [N/(rad/s)^2]
double Cq_ = 2.17e-007; // Drag coefficient [Nm/(rad/s)^2]
double Ixx_ = 2.04016e-003; // Inertia around the X axis [Kg/m^3]
double Iyy_ = 1.5677e-003; // Inertia around the Y axis [Kg/m^3]
double Izz_ = 3.51779e-003; // Inertia around the Z axis [Kg/m^3]
double m_ = 0.4305; // Mass of the quadrotor [Kg]
double d_ = 0.35; // Distance from the rotor to the mass center of the quadrotor [m]
double Jr_ = 1.66e-005; // Inertia of a unitary rotor (approximated as a disc) [Kg*m^2] //FIXME Inertia 
double ts_ = 0.0083; // Sampling time
double g_ = 9.81; // Acceleration of gravity
int num_states_ = 12;
int num_inputs_ = 4;


std::string getWorkingPath( void ) {
	char buff[PATH_MAX];
	getcwd( buff, PATH_MAX );
	std::string cwd( buff );
	return cwd;
}

bool writeToDisc()
{
	bool data = true;
	if (u_[0].size() == 0) {
		ROS_WARN("Could not save the data because the input signal information is NULL");
		data = false;
	}
	if (x_[0].size() == 0) {
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
		
		for (unsigned int j = 0; j < u_[0].size(); j++) {
			infile << time_[j];
			for (unsigned int i = 0; i < u_.size(); i++) {
				infile << '\t';
				infile << u_[i][j];
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
    	for (unsigned int j = 0; j < x_[0].size(); j++) {
			outfile << time_[j];
			for (unsigned int i = 0; i < x_.size(); i++) {
				outfile << '\t';
				outfile << x_[i][j];
			}
			outfile	<< std::endl;
		}
		outfile.close();
		ROS_INFO("Output data validation file recorded.");

		std::ofstream nonlinearfile;
		nonlinearfile.open(nonlinear_data_name.c_str());
		nonlinearfile.precision(4);
		nonlinearfile.setf(std::ios::fixed, std::ios::floatfield);
		nonlinearfile.setf(std::ios::left, std::ios::adjustfield);

		nonlinearfile << "t\t" << "x\t" << "y\t" << "z\t" << "u\t" << "v\t" << "w\t" << "roll\t" << "pitch\t" << "yaw\t" << "p\t" << "q\t" << "r" << std::endl;
    	for (unsigned int j = 0; j < x_NL_[0].size(); j++) {
			nonlinearfile << time_[j];
			for (unsigned int i = 0; i < x_NL_.size(); i++) {
				nonlinearfile << '\t';
				nonlinearfile << x_NL_[i][j];
			}
			nonlinearfile	<< std::endl;
		}
		nonlinearfile.close();
		ROS_INFO("Nonlinear output data validation file recorded.");
	}
	std::string path = getWorkingPath();
	std::cout << path;
	return true;
}


void computeLTIModel(Eigen::MatrixXd &A, Eigen::MatrixXd &B, double* state_op_vect, double* input_op_vect)
{
	Eigen::Map<Eigen::VectorXd> x_bar(state_op_vect, num_states_);
	Eigen::Map<Eigen::VectorXd> u_bar(input_op_vect, num_inputs_);

	double phi = x_bar(6);
	double theta = x_bar(7);
	double psi = x_bar(8);
	double p = x_bar(9);
	double q = x_bar(10);
	double r = x_bar(11);
	double U1 = u_bar(0);
	
	
	A(0,3) = ts_;
	A(1,4) = ts_;
	A(2,5) = ts_;
	A(3,6) = ts_ * (sin(psi) * cos(phi) - cos(psi) * sin(theta) * sin(phi)) * U1 / m_;
	A(3,7) = ts_ * (cos(psi) * cos(theta) * cos(phi)) * U1 / m_;
	A(3,8) = ts_ * (cos(psi) * sin(phi) - sin(psi) * sin(theta) * cos(psi)) * U1 / m_;
	A(4,6) = - ts_ * (sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi)) * U1 / m_;
	A(4,7) = ts_ * (sin(psi) * cos(theta) * cos(psi)) * U1 / m_;
	A(4,8) = ts_ * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) * U1 / m_;
	A(5,6) = - ts_ * (cos(theta) * sin(phi)) * U1 / m_;
	A(5,7) = - ts_ * (sin(theta) * cos(phi)) * U1 / m_;	
	A(6,6) = ts_ * (q * cos(phi) - r * sin(phi)) * tan(theta);
	A(6,7) = ts_ * (q * sin(phi) + r * cos(phi)) / (cos(theta) * cos(theta));
	A(6,9) = ts_;
	A(6,10) = ts_ * sin(phi) * tan(theta);
	A(6,11) = ts_ * cos(phi) * tan(theta);
	A(7,6) = - ts_ * (q * sin(phi) + r * cos(phi));
	A(7,10) = ts_ * cos(phi);
	A(7,11) = - ts_ * sin(phi);
	A(8,6) = ts_ * (q * cos(phi) - r * sin(phi)) / cos(theta);
	A(8,7) = ts_ * (q * sin(phi) + r * cos(phi)) * tan(theta) / cos(theta);
	A(8,10) = ts_ * sin(phi) / cos(theta);
	A(8,11) = ts_ * cos(phi) / cos(theta);
	A(9,10) = ts_ * r * (Iyy_ - Izz_) / Ixx_;
	A(9,11) = ts_ * q * (Iyy_ - Izz_) / Ixx_;
	A(10,9) = ts_ * r * (Izz_ - Ixx_) / Iyy_;
	A(10,11) = ts_ * p * (Izz_ - Ixx_) / Iyy_;
	A(11,9) = ts_ * q * (Ixx_ - Iyy_) / Izz_;
	A(11,10) = ts_ * p * (Ixx_ - Iyy_) / Izz_;
	
	B(3,0) = ts_ * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) / m_;		
	B(4,0) = ts_ * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)) / m_;	
	B(5,0) = ts_ * cos(theta) * cos(phi) / m_;
	B(9,1) = ts_ * d_ / Ixx_;
	B(10,2) = ts_ * d_ / Iyy_;
	B(11,3) = ts_ / Izz_;
}


double* simulatePlant(double *state_vect, double *input_vect, double *state_op_vect, double *input_op_vect, double sampling_time)
{
	Eigen::Map<Eigen::VectorXd> x(state_vect, num_states_);
	Eigen::Map<Eigen::VectorXd> x_bar(state_op_vect, num_states_);
	Eigen::Map<Eigen::VectorXd> u(input_vect, num_inputs_);
	Eigen::Map<Eigen::VectorXd> u_bar(input_op_vect, num_inputs_);
	
	double phi = x_bar(6);
	double theta = x_bar(7);
	u_bar(0) = g_ * m_ / (cos(phi) * cos(theta));
	
	Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_states_, num_states_);
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num_states_, num_inputs_);
	
	computeLTIModel(A, B, state_op_vect, input_op_vect);
	
	x = x_bar + A * (x - x_bar) + B * (u - u_bar);
		
	return state_vect;
}

double* simulateNonLinearPlant(double *current_state, double *current_input)
{
	//ROS_ASSERT(sizeof(current_state) == num_states_);

	//ROS_ASSERT(sizeof(current_input) == num_inputs_);

	// Map into Eigen objects for easier manipulation	
	
	Eigen::Map<Eigen::VectorXd> x_current(current_state, 12, 1);
	Eigen::Map<Eigen::VectorXd> u_current(current_input, 4, 1);	
	Eigen::VectorXd x_new = Eigen::MatrixXd::Zero(num_states_, 1);

	// Solve the difference equations recursively

	x_new(0) = x_current(0) + ts_*x_current(3);

	x_new(1) = x_current(1) + ts_*x_current(4);

	x_new(2) = x_current(2) + ts_*x_current(5);

	x_new(3) = x_current(3) + (ts_/m_)*(cos(x_current(8))*sin(x_current(7))*cos(x_current(6)) + sin(x_current(8))*sin(x_current(6)))*u_current(0);

	x_new(4) = x_current(4) + (ts_/m_)*(sin(x_current(8))*sin(x_current(7))*cos(x_current(6)) - cos(x_current(8))*sin(x_current(6)))*u_current(0);

	x_new(5) = x_current(5) + ts_*(-g_ + cos(x_current(7))*cos(x_current(6))*(u_current(0)/m_));

	x_new(6) = x_current(6) + ts_*(x_current(9) + x_current(10)*sin(x_current(6))*tan(x_current(7)) + x_current(11)*cos(x_current(6))*tan(x_current(8)));

	x_new(7) = x_current(7) + ts_*(x_current(10)*cos(x_current(6)) - x_current(11)*sin(x_current(6)));

	x_new(8) = x_current(8) + (ts_/cos(x_current(7)))*(x_current(10)*sin(x_current(6)) + x_current(11)*cos(x_current(6)));

	x_new(9) = x_current(9) + ts_*( (Iyy_ - Izz_)*((x_current(10)*x_current(11))/Ixx_) + (d_/Ixx_)*u_current(1));

	x_new(10) = x_current(10) + ts_*( (Izz_ - Ixx_)*((x_current(9)*x_current(11))/Iyy_) + (d_/Iyy_)*u_current(2));

	x_new(11) = x_current(11) + ts_*( (Ixx_ - Iyy_)*((x_current(9)*x_current(10))/Izz_) + (1/Izz_)*u_current(3));

	std::cout << "States at K+1:\t" << x_new.transpose() << std::endl; 
	x_current = x_new;
	return current_state;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "model_validation");
	
	double current_time;

	/** Linearized model initial values and operation points */
	double state[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double input[4] = {0.0, 0.0, 0.0, 0.0};
	double state_op[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double input_op[4] = {0.0, 0.0, 0.0, 0.0};
	double *new_state;
	Eigen::Map<Eigen::VectorXd> x(&state[0], num_states_);
	Eigen::Map<Eigen::VectorXd> u(&input[0], num_inputs_);
	Eigen::Map<Eigen::VectorXd> x_new(new_state, num_states_);

	/** Nonlinear simulator initial values */
	double *new_NL_state;
	double NL_state[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};	
	Eigen::Map<Eigen::VectorXd> x_NL_new(new_NL_state, num_states_);
	Eigen::Map<Eigen::VectorXd> x_NL(&NL_state[0], num_states_);
	

	u_.resize(4);
	x_.resize(12);
	x_NL_.resize(12);
	time_.push_back(0.0);
	for (int j = 0; j < num_inputs_; j++) {
		u_[j].push_back(u(j));
	}
	for (int j = 0; j < num_states_; j++) {
		x_[j].push_back(x(j));
		x_NL_[j].push_back(x_NL(j));
 	}
	
	
	/** Simulation **/
	for (unsigned long int i = 0; i < 1000; i++) {
		current_time = i * ts_;
		
		/** INPUT SELECTION **/
		if (current_time > 0.0 && current_time < 2.) {
			u(0) = 2. * g_ * m_;
		}
		else if (current_time > 4. && current_time < 6.0) {
			u(0) = g_ * m_;
		}
		else {
			u(0) = 0.0;
		}
				
		/** STATE SPACE FORMULATION AND CALCULATION OF THE NEXT STEP **/
		new_state = simulatePlant(state, input, state_op, input_op, ts_);
		for (int j = 0; j < num_states_; j++) {
			state[j] = new_state[j];
			x_[j].push_back(x(j));
		}

		new_NL_state = simulateNonLinearPlant(NL_state, input);
		for (int j = 0; j < num_states_; j++) {
			NL_state[j] = new_NL_state[j];
			x_NL_[j].push_back(x_NL(j));
		}
				
		/** WRITING TO RECORDING VECTORS **/
		for (int j = 0; j < num_inputs_; j++) {
			u_[j].push_back(u(j));
		}		
		time_.push_back(current_time);
		
		i++;
	}

	bool write = false;
	write = writeToDisc();
	if (write)
		std::cout << "Satisfactory recorded validation data." << std::endl;

	return 0;
}
