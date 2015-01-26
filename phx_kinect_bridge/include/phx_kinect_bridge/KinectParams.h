/*
 * KinectParams.h
 *
 *  Created on: 13.07.2014
 *      Author: ic3
 */

#ifndef KINECTPARAMS_H_
#define KINECTPARAMS_H_

#include <ros/ros.h>

struct KinectParams {
public:

	/**
	 * Name of the ROS topic for the activate messages
	 */
	static const std::string TOPIC_ACTIVATE;

	/**
	 * Name of the ROS topic for the command messages
	 */
	static const std::string TOPIC_COMMAND;

	/**
	 * Name of the ROS topic for the motor-command messages
	 */
	static const std::string TOPIC_COMMAND_MOTOR;

	/**
	 * Name of the ROS parameter for the port where the Kinect sends the messages to.
	 */
	static const std::string PORT;
};

#endif /* KINECTPARAMS_H_ */
