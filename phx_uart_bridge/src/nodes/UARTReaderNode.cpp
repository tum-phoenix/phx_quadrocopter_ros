/*
 * UARTReaderNode.cpp
 *
 *  Created on: 04.12.2013
 *      Author: ic3
 */

#include <phx_uart_bridge/MessageSerializer.h>
#include <phx_uart_bridge/SensorMessage.h>
#include <phx_uart_bridge/UARTBridgeParams.h>
#include <phx_uart_bridge/UARTInterface.h>

/**
 * Goal of this node is to write packages from ROS to the UART interface.
 */
int main(int argc, char **argv) {

	ROS_INFO("Starting UART Reader node");

	// Initialize ROS
	ros::init(argc, argv, "UARTReaderNode");
	ros::NodeHandle nh;

	// Sensor message is a pointer, because the only subscriber resides on the same node
	phx_uart_bridge::SensorMessagePtr sensorMessage;
	ros::Publisher pub = nh.advertise<phx_uart_bridge::SensorMessage>(UARTBridgeParams::TOPIC_SENSOR_RAW, 1);

	// Create and open the UART interface
	UARTInterface uart;
	uart.openUART(nh, ReadOnly);
	char buffer[18];

	// Start the loop
	while (ros::ok()) {

		// Wait for start flag
		while (uart.readChar() != 'U')
			continue;

		// Read the whole message
		char * p = & buffer[0];
		if (!uart.readString(p, 17)) {
			ROS_ERROR("Failed to read from UART");
			continue;
		}

		// Reads the message byte-wise
//		for (int i = 0; i < 18; i++)
//			buffer[i] = uartInterface.readChar();


		// Deserialize the message and update the object
		MessageSerializer::deserialize(sensorMessage, buffer);

		// Publish it to the ROS world
		pub.publish(sensorMessage);

		// Give some processing time to other threads
		ros::spinOnce();
	}

	// Eventually close the UART interface
	uart.closeUART();

	return 0;
}
