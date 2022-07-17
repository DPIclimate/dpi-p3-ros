#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <ncurses.h>
#include <sstream>

class Controller{
	public:
		void motor_state(ros::NodeHandle* node, bool enable = true);
};
