/*
 * KinectPubStubNode.cpp
 *
 *  Created on: 18.10.2013
 *      Author: ic3
 */

#include <phx_port_interface/PortSender.h>
#include <phx_kinect_bridge/KinectBridge.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {

	// Initialize ROS
	ros::init(argc, argv, "KinectPubStub");
	ros::NodeHandle nodeHandle;

	// Create the PortSender
	std::string port;
	nodeHandle.param<std::string>(KinectParams::PORT, port, std::string("31339"));
	PortSender sender(port.c_str());

	// Enter the loop and send the different commands, one after another
	int cmd = 1;
	while (ros::ok()) {
		for (int i = 0; i < 25 && ros::ok(); i++) {
			ROS_INFO("Sending kinect command: %d", cmd);
			sender.sendMessage(&cmd, 1);
			ros::Duration(.2).sleep();
		}
		cmd = (cmd % 6) + 1;
	}

	ROS_INFO("KinectPubStubNode terminated");
	return 0;
}

