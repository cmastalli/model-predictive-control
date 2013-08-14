#include <ros/ros.h>
#include <std_msgs/String.h>
//#include "tum_ardrone/filter_state.h"
#include <gazebo_msgs/ModelStates.h>
#include <ardrone_autonomy/Navdata.h>
#include <vector>
#include <fstream>
#include <geometry_msgs/Twist.h>


std::string input_data_name_ = "/home/adminmecatronica/ros_workspace/model-predictive-control/ardrone_mpc/data/identification_input_data.txt";
std::string output_data_name_ = "/home/adminmecatronica/ros_workspace/model-predictive-control/ardrone_mpc/data/identification_output_data.txt";
std::string output_data_rotz_name_ = "/home/adminmecatronica/ros_workspace/model-predictive-control/ardrone_mpc/data/identification_output_data_rotz.txt";

std::vector<std::vector<double> > input_;
std::vector<std::vector<double> > estimated_states_;
std::vector<double> estimated_states_2;
std::vector<double> t_input_;
std::vector<double> t_output_lin_;
std::vector<double> t_output_ang_;
double t_start_;


void outputLinearPositionDataCallback(const gazebo_msgs::ModelStates& msg)
{
	t_output_lin_.push_back(ros::Time::now().toSec() - t_start_);
	
	estimated_states_[0].push_back(msg.pose[2].position.x);
	estimated_states_[1].push_back(msg.pose[2].position.y);
	estimated_states_[2].push_back(msg.pose[2].position.z);

}

void outputAngularPositionVelocityDataCallback(const ardrone_autonomy::Navdata& msg)
{
	t_output_ang_.push_back(ros::Time::now().toSec() - t_start_);
	
	estimated_states_2.push_back(msg.rotZ);

}


void inputDataCallback(const geometry_msgs::Twist& msg)
{
	t_input_.push_back(ros::Time::now().toSec() - t_start_);
	
	input_[0].push_back(msg.linear.x);
	input_[1].push_back(msg.linear.y);	
	input_[2].push_back(msg.linear.z);
	input_[3].push_back(msg.angular.z);
}


void writeToDisc()
{		
	std::ofstream outfile;
	outfile.open(input_data_name_.c_str());
	outfile.precision(4);
	outfile.setf(std::ios::fixed, std::ios::floatfield);
	outfile.setf(std::ios::left, std::ios::adjustfield);
	for (unsigned int j = 0; j < input_[0].size(); j++) {
		if (j == 0) {
			for (unsigned int i = 0; i < input_.size() + 1; i++) {
				if (i == 0)
					outfile << "t" << '\t';
				if (i == input_.size())
					outfile << "u_" << i+1 << std::endl;
				else
					outfile << "u_" << i+1 << '\t';
			}
		}
   		for (unsigned int i = 0; i < input_.size() + 1; i++) {
			if (i == 0)
				outfile << t_input_[j] << '\t';
			if (i == input_.size())
				outfile << input_[i][j] << std::endl;
			else
				outfile << input_[i][j] << '\t';
		}
	}
	outfile.close();
	
	
	outfile.open(output_data_name_.c_str());
	outfile.precision(4);
	outfile.setf(std::ios::fixed, std::ios::floatfield);
	outfile.setf(std::ios::left, std::ios::adjustfield);
	for (unsigned int j = 0; j < estimated_states_[0].size(); j++) {
		if (j == 0) {
			outfile << "t" << '\t' << "x" << '\t' << "y" << '\t' << "z"  << '\t' << "yaw" << std::endl;
		}
   	 	for (unsigned int i = 0; i < estimated_states_.size() + 2; i++) {
    			if (i == 0)
    				outfile << t_output_ang_[j] << '\t';
			if (i == estimated_states_.size())
				outfile << estimated_states_[i][j] << std::endl;
			else
				outfile << estimated_states_[i][j] << '\t';
		}
	}
	outfile.close();

	outfile.open(output_data_rotz_name_.c_str());
	outfile.precision(4);
	outfile.setf(std::ios::fixed, std::ios::floatfield);
	outfile.setf(std::ios::left, std::ios::adjustfield);
	for (unsigned int j = 0; j < estimated_states_2.size() + 1; j++) {
		if (j == 0) {
			outfile << "t" << '\t' << "yaw" << std::endl;
		}
		else {
			outfile << t_output_ang_[j] << '\t' << estimated_states_2[j] << std::endl;
		}
	}
	outfile.close();
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "ardrone_record");

	ros::NodeHandle nh;

	estimated_states_.resize(3);
	input_.resize(4);
	t_start_ = ros::Time::now().toSec();

	ros::Subscriber output_linear_data = nh.subscribe("gazebo/ModelStates", 100, outputLinearPositionDataCallback);
	ros::Subscriber output_angular_data = nh.subscribe("ardrone/Navdata", 100, outputAngularPositionVelocityDataCallback);
	ros::Subscriber input_data = nh.subscribe("cmd_vel", 100, inputDataCallback);


	ros::spin();

	if (!ros::ok()) {
		writeToDisc();
	}

  return 0;
}
