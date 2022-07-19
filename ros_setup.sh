#!/usr/bin/env bash

# Use this script of you want to keep your sanity.
# It will source catkin dependencies and then run cmake across all
# packages in your catkin workspace.

# Optionally pass "-r" to refresh / update your network configuration
# of connected ROS nodes.

# Run 
echo "[ROS_SETUP] Souring devel/setup.bash"
source ~/catkin_ws/devel/setup.bash

# Run catkin_make
echo "[ROS_SETUP] Running catkin_make"
cd ~/catkin_ws
catkin_make
cd -

if [ $# -ne 2 ]; then 
	if [ "$1" == "-r" ]; then
		echo "[ROS_SETUP] Setting IP of robot and workstation"

		## As http://<ip_of_robot>:11311
		export ROS_MASTER_URI=http://192.168.0.12
		## As <ip_of_computer>
		export ROS_HOSTNAME=192.168.0.12
		## As <ip_of_computer>
		export ROS_IP=192.168.0.9
	fi
fi
