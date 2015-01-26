/*
 * UARTMessageMergerNode.cpp
 *
 *  Created on: 15.07.2014
 *      Author: ic3
 */

#include <phx_uart_bridge/MotorMessageMerger.h>
#include <phx_uart_bridge/SensorMessageMerger.h>

#include <ros/ros.h>

/**
 * Merges the raw motor message with given overrides.
 */
int main(int argc, char **argv) {

	ROS_INFO("Starting Motor Message Merger Node");

	// Initialize ROS
	ros::init(argc, argv, "UARTMessageMergerNode");
	ros::NodeHandle nh("UARTMessageMergerNode");

	MotorMessageMerger motorMessageMerger(nh);
	SensorMessageMerger sensorMessageMerger(nh);
	ros::spin();
}
