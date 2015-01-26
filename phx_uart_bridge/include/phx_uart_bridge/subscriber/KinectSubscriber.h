/*
 * KinectSubscriber.h
 *
 *  Created on: 16.10.2013
 *      Author: ic3
 */

#ifndef KINECTSUBSCRIBER_H_
#define KINECTSUBSCRIBER_H_

#include <phx_uart_bridge/MessageOverrider.h>
#include <phx_uart_bridge/MotorMessage.h>
#include <phx_uart_bridge/SensorMessage.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

class KinectSubscriber : public MessageOverrider {

public:

	KinectSubscriber(ros::NodeHandle & nh);
	virtual ~KinectSubscriber() { }

	bool overrideSensor(phx_uart_bridge::SensorMessage & sensorMessage) const;
	bool overrideMotor(phx_uart_bridge::MotorMessage & motorMessage) const;

private:

	// Only true if KinectSubscriber wants to override data
	bool isEnabled;

	/**
	 * Last message received from the ROS
	 */
	phx_uart_bridge::SensorMessage sensorMsg;

	/**
	 * TODO Remove
	 *
	 * Tag-der-offenen-Tür-Override
	 */
	phx_uart_bridge::MotorMessage motorMsg;

	/**
	 * Callback for the incoming ROS ActivateMessages
	 */
	void processActivateMessageCb(const std_msgs::String& msg);

	/**
	 * Callback for the incoming ROS SensorMessages
	 */
	void processSensorMessageCb(const phx_uart_bridge::SensorMessage& msg);

	/**
	 * Callback for the incoming ROS MotorMessages
	 */
	void processMotorMessageCb(const phx_uart_bridge::MotorMessage& msg);

};

#endif /* KINECTSUBSCRIBER_H_ */
