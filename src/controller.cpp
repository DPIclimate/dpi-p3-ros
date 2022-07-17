#include "controller.hpp"

void Controller::motor_state(ros::NodeHandle* node, bool enable){

	std_msgs::String msg;
	msg.data = "Enabled motors";

	ros::Publisher motors = node->
		advertise<std_srvs::Empty::Request>("RosAria/enable_motors", 1);

	if(!enable){
		msg.data = "Disabled motors";

		motors = node->
			advertise<std_srvs::Empty::Request>("RosAria/disable_motors", 1);
	} 

	motors.publish(msg);

	ROS_INFO("%s", msg.data.c_str());
}

