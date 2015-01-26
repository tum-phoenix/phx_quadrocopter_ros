/*
 * SimuLink_ReceiverStub.cpp
 *
 *  Created on: 22.09.2013
 *      Author: ic3
 */

#include <ros/ros.h>
#include <phx_port_interface/PortListener.h>


int main(int argc, char **argv) {

	// Initialize ROS
	ros::init(argc, argv, "SimuLink_ReceiverStub", ros::init_options::AnonymousName);
	ros::NodeHandle nodeHandle;

	// Create the listener
	std::string port;
	nodeHandle.param("/SimuLink_ReceiverStub/port", port, std::string("31338"));
	PortListener listener(port.c_str());

	// Listen to the port and print the messages
	ROS_INFO("Listen for SensorMessages on port %s", port.c_str());
	short buffer[11];
	int len = -1;
	while (ros::ok() && (len = listener.readNext(buffer)) > 0)
		ROS_INFO("%5d | %5d | %5d | %5d | %5d | %5d | %5d | %5d | %5d | %5d | %5d",
				buffer[0], buffer[1], buffer[2], buffer[3],
				buffer[4], buffer[5], buffer[6],
				buffer[7], buffer[8], buffer[9],
				buffer[10]);

	return 0;
}
