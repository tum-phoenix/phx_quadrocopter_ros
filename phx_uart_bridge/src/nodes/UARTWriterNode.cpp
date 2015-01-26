/*
 * UARTReaderNode.cpp
 *
 *  Created on: 04.12.2013
 *      Author: ic3
 */

#include <phx_uart_bridge/MessageSerializer.h>
#include <phx_uart_bridge/MotorMessage.h>
#include <phx_uart_bridge/UARTBridgeParams.h>
#include <phx_uart_bridge/UARTInterface.h>

#include <ros/ros.h>

void MotorMessageCb(const phx_uart_bridge::MotorMessagePtr& msg);
phx_uart_bridge::MotorMessage motorMessage;
UARTInterface uart;
char buffer[18];

/**
 * Goal of this node is to publish messages from the UART interface to the ROS world.
 */
int main(int argc, char **argv) {
	ROS_INFO("Starting UART Writer node");

	// Initialize ROS
	ros::init(argc, argv, "UARTWriterNode");
	ros::NodeHandle nh;

	// Create and open the UART interface
	uart.openUART(nh, WriteOnly);

	nh.subscribe(UARTBridgeParams::TOPIC_MOTOR_RAW, 1, MotorMessageCb);
	ros::spin();

	// Eventually close the UART interface
	uart.closeUART();

	return 0;
}

void MotorMessageCb(const phx_uart_bridge::MotorMessagePtr& msg) {

	// Apply the new values
	motorMessage.motor0 = msg->motor0;
	motorMessage.motor1 = msg->motor1;
	motorMessage.motor2 = msg->motor2;
	motorMessage.motor3 = msg->motor3;

	// Write the response back to UART
	const int size = MessageSerializer::serialize(motorMessage, buffer);
	const char * p = &buffer[0];
	if (!uart.writeString(p, size))
		ROS_ERROR("Failed to write UART package");
}
