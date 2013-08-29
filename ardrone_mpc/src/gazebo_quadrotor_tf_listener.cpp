#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <turtlesim/Velocity.h>
//#include <turtlesim/Spawn.h>
#include <Eigen/Dense>
#include <geometry_msgs/Twist>

int main(int argc, char** argv){
  ros::init(argc, argv, "gazebo_tf_listener");

  ros::NodeHandle node;

  /*ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = 
       node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);*/

  ros::Publisher quad_states = node.advertise<geometry_msgs::Twist>("/transformation", 1);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
		// Here we are checking for the status of the transformation from world coordinates to the mobile frame in the quadrotor
		listener.lookupTransform("/nav", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    turtlesim::Velocity vel_msg;
    vel_msg.angular = 4 * atan2(transform.getOrigin().y(),
                                transform.getOrigin().x());
    vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                 pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
