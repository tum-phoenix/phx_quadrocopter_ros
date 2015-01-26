/*
 * KinectPubStubNode.cpp
 *
 *  Created on: 18.10.2013
 *      Author: ic3
 */

#include <phx_uart_bridge/subscriber/KinectSubscriber.h>
#include <phx_uart_bridge/SensorMessage.h>
#include <phx_uart_bridge/MotorMessage.h>

#include <ros/ros.h>

/**
 * Goal of this node is to simply print the Kinect-data received from the KinectSubscriber.
 */
int main(int argc, char **argv) {

	// Initialize ROS
	ros::init(argc, argv, "KinectSubscriberNode");
	ros::NodeHandle nodeHandle;

	// Create the KinectListener
	KinectSubscriber sub(nodeHandle);
	phx_uart_bridge::SensorMessage sensorMessage;
	phx_uart_bridge::MotorMessage motorMessage;

	// Enter the loop and send the different commands, one after another
	int cmd = 1;
	while (ros::ok()) {

		sub.overrideSensor(sensorMessage);
		ROS_INFO("R0 %4d | R1 %4d | R2 %4d | R3 %4d", sensorMessage.radio0, sensorMessage.radio1, sensorMessage.radio2, sensorMessage.radio3);

		ros::spinOnce();
		ros::Rate(.2).sleep();

	}

	ROS_INFO("KinectSubscriberNode terminated");
	return 0;
}

