#include "controller.h"

int main(int argc, char* argv[]){

	initscr();
	raw();
	keypad(stdscr, TRUE);
	noecho();
	
	// Initialise ROS with cmd line arguments
	ros::init(argc, argv, "talker");

	// Node hander
	ros::NodeHandle node;

	// Create publisher (talker)
	// fmt ("<topic>", <size_of_queue>)
	ros::Publisher chatter_pub = node.
		advertise<geometry_msgs::Twist>("AMRISim/cmd_vel", 1000);

	// Refresh rate
	ros::Rate loop_rate(10);

	// Set velocity (should spin the robot round and round)
	geometry_msgs::Twist vel;
	vel.linear.y = 0.0;
	vel.linear.z = 0.0;

	vel.angular.x = 0.0;
	vel.angular.y = 0.0;

	double lin_x = 0.0;
	double ang_z = 0.0;

	int c = 0;
	while(ros::ok()){
		c = 0;

		ROS_INFO("Moving...\n");

		switch(c = getch()){
			case KEY_UP: // Up key
				lin_x = 1.0;
				break;
			case KEY_DOWN: // Down
				lin_x = -1.0;
				break;
			case KEY_LEFT: // Left
				ang_z = 1.0;
				break;
			case KEY_RIGHT: // Right
				ang_z = -1.0;
				break;
			default:
				break;
				return 0;
		}

		vel.linear.x = lin_x;
		vel.angular.z = ang_z;

		// Publish message
		chatter_pub.publish(vel);

		ros::spinOnce();

		loop_rate.sleep();

		lin_x = 0.0;
		ang_z = 0.0;

		refresh();
	}
	getch();
	endwin();

	return 0;
}
