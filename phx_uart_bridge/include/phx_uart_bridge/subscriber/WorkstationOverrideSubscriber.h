/*
 * WorkstationOverrideSubscriber.h
 *
 *  Created on: 28.10.2013
 *      Author: ic3
 */

#ifndef WORKSTATIONOVERRIDESUBSCRIBER_H_
#define WORKSTATIONOVERRIDESUBSCRIBER_H_

#include <phx_uart_bridge/MessageOverrider.h>
#include <phx_uart_bridge/MotorMessage.h>
#include <phx_uart_bridge/SensorMessage.h>

#include <ros/ros.h>
#include <std_msgs/String.h>


/**
 * Subscriber for the override publisher running on a laptop.
 */
class WorkstationOverrideSubscriber : public MessageOverrider {
public:

	WorkstationOverrideSubscriber(ros::NodeHandle & nh);

    /**
     * @return true if data has been overridden
     */
    bool overrideSensor(phx_uart_bridge::SensorMessage & msg) const;

    /**
     * @return true if data has been overridden
     */
    bool overrideMotor(phx_uart_bridge::MotorMessage & msg) const;

private:

	ros::Subscriber activationSubscriber;
	ros::Subscriber sensorSubscriber;
	ros::Subscriber motorSubscriber;

	bool isSensorActive;
	phx_uart_bridge::SensorMessage sensorMessage;

	bool isMotorActive;
	phx_uart_bridge::MotorMessage motorMessage;

	void processSensorMessageCb(const phx_uart_bridge::SensorMessage& msg);

	void processMotorMessageCb(const phx_uart_bridge::MotorMessage& msg);

	void processActivationMessageCb(const std_msgs::String& msg);

};

#endif /* WORKSTATIONOVERRIDESUBSCRIBER_H_ */
