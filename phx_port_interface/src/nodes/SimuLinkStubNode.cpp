#include <phx_port_interface/PortListener.h>
#include <phx_port_interface/PortSender.h>
#include <ros/ros.h>

/*
 * Simulate SimuLink controller
 *
 * DEPRECATED: Use launch-file instead
 */
int main(int argc, char **argv) {

	// Initialize ROS
	ros::init(argc, argv, "SimuLinkStubNode");
	ros::NodeHandle nodeHandle;

	// Create the listener
	std::string listenPort;
	nodeHandle.param(TUMPHOENIX_PORTLISTENER_PORT_PARAM, listenPort, std::string("31338"));
	PortListener listener(listenPort.c_str());

	// Create the PortSender
	std::string targetPort;
	nodeHandle.param(TUMPHOENIX_PORTSENDER_PORT_PARAM, targetPort, std::string("31337"));
	PortSender sender(targetPort.c_str());

	// Create the buffers
	short sensorBuffer[11];
	char simuLinkBuffer[9];
	int val = 1000;
	simuLinkBuffer[0] = val;
	simuLinkBuffer[1] = val >> 8;
	simuLinkBuffer[2] = val;
	simuLinkBuffer[3] = val >> 8;
	simuLinkBuffer[4] = val;
	simuLinkBuffer[5] = val >> 8;
	simuLinkBuffer[6] = val;
	simuLinkBuffer[7] = val >> 8;
	simuLinkBuffer[8] = '\0';

	// Enter the loop and provide some fake SimuLink behaviour
	while (ros::ok()) {

		// Receive sensor data
		if (listener.readNext(sensorBuffer) < 0)
			ROS_WARN("Failed to receive sensor data");
		else
			ROS_INFO("%4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d",
					sensorBuffer[0], sensorBuffer[1], sensorBuffer[2],
					sensorBuffer[3], sensorBuffer[4], sensorBuffer[5],
					sensorBuffer[6], sensorBuffer[7], sensorBuffer[8],
					sensorBuffer[9], sensorBuffer[10]);

		// Send fake-data
		sender.sendMessage(simuLinkBuffer, 8);
	}
}
