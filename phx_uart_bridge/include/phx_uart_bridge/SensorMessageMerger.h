/*
 * SensorMessageMerger.h
 *
 *  Created on: 15.07.2014
 *      Author: ic3
 */

#ifndef SENSORMESSAGEMERGER_H_
#define SENSORMESSAGEMERGER_H_

#include <phx_uart_bridge/SensorMessage.h>

#include <ros/ros.h>

/**
 * Merges the raw sensor message with given overrides.
 */
class SensorMessageMerger {

public:

	ros::Publisher publisher;

	SensorMessageMerger(ros::NodeHandle & nh);

private:

	std::vector<MessageOverrider> overriders;
	phx_uart_bridge::SensorMessage msg;

	/**
	 * Callback for the raw sensor message
	 */
	void rawSensorMessageCb(const phx_uart_bridge::SensorMessagePtr & msg);

};

#endif /* SENSORMESSAGEMERGER_H_ */
