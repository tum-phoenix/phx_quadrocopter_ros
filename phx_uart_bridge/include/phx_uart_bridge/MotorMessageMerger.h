/*
 * MotorMessageMerger.h
 *
 *  Created on: 15.07.2014
 *      Author: ic3
 */

#ifndef MOTORMESSAGEMERGER_H_
#define MOTORMESSAGEMERGER_H_

#include <phx_uart_bridge/MessageOverrider.h>
#include <phx_uart_bridge/MotorMessage.h>

#include <ros/ros.h>

/**
 * Merges the raw motor message with given overrides.
 */
class MotorMessageMerger {

public:

	ros::Publisher publisher;

	MotorMessageMerger(ros::NodeHandle & nh);

private:

	std::vector<MessageOverrider> overriders;
	phx_uart_bridge::MotorMessage msg;

	/**
	 * Callback for the raw motor message
	 */
	void rawMotorMessageCb(const phx_uart_bridge::MotorMessagePtr & msg);

};

#endif /* MOTORMESSAGEMERGER_H_ */
