#include "main.hpp"

Controller controller;

int main(int argc, char* argv[]){

	// Initialise ROS with cmd line arguments
	ros::init(argc, argv, "DPIPioneer3");

	// Node hander
	ros::NodeHandle node;

	controller.motor_state(&node, true);

	// Create publisher (talker)
	// fmt ("<topic>", <size_of_queue>)
	ros::Publisher chatter_pub = node.
		advertise<geometry_msgs::Twist>("AMRISim/cmd_vel", 1);

	// Refresh rate
	ros::Rate loop_rate(10);

	// Set velocity (should spin the robot round and round)
	geometry_msgs::Twist vel;
	vel.linear.y = 0.0;
	vel.linear.z = 0.0;

	vel.angular.x = 0.0;
	vel.angular.y = 0.0;

	double lin_x = 1.0;
	double ang_z = 0.0;

	while(ros::ok()){
		std_msgs::String msg;
		std::stringstream ss;

		// Logging
		ss << "Moving: Linear X: " << lin_x << " Angular Z: " << ang_z;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());

		vel.linear.x = lin_x;
		vel.angular.z = ang_z;

		// Publish message
		chatter_pub.publish(vel);

		ros::spinOnce();

		loop_rate.sleep();

	}
	endwin();

	return 0;
}

