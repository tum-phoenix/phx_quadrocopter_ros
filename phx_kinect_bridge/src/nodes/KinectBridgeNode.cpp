/*
 * KinectBridge.cpp
 *
 *  Created on: 07.11.2013
 *      Author: ic3
 */

#include <phx_kinect_bridge/KinectBridge.h>
#include <ros/ros.h>

/**
 * The goal of this node is to publish the Kinect-data received via UDP to ROS.
 */
int main(int argc, char **argv) {

	// Initialize ROS
	ros::init(argc, argv, "KinectBridgeNode");
	ros::NodeHandle nodeHandle("KinectBridgeNode");

	// Get parameters and create the KinectBridge
	KinectBridge kinectBridge(nodeHandle);
	kinectBridge.startBridge();
}