/*
 * PortSenderNode.cpp
 *
 *  Created on: 22.09.2013
 *      Author: ic3
 */

#include <ros/ros.h>
#include <ros/rate.h>
#include <phx_port_interface/PortSender.h>

int main(int argc, char **argv) {

	// Initialize ROS
	ros::init(argc, argv, "SimuLink_SenderStub", ros::init_options::AnonymousName);
	ros::NodeHandle nodeHandle;

	// Create the PortSender
	std::string port;
	nodeHandle.param<std::string>("/SimuLink_SenderStub/port", port, std::string("31337"));
	PortSender sender(port.c_str());

	// Create the buffers
	char buffer[9];
	int val = 1000;
	buffer[0] = val;
	buffer[1] = val >> 8;
	buffer[2] = val;
	buffer[3] = val >> 8;
	buffer[4] = val;
	buffer[5] = val >> 8;
	buffer[6] = val;
	buffer[7] = val >> 8;
	buffer[8] = '\0';

    // Send this package with a 10Hz frequency
    ros::Rate rate(10);
	ROS_INFO("Starting to send packages. Frequency: %d.%d", rate.cycleTime().sec, rate.cycleTime().nsec);
	while ( ros::ok() ) {
		sender.sendMessage(buffer, 8);
		rate.sleep();
	}

	ROS_INFO("Closing node");
	return 0;
}
