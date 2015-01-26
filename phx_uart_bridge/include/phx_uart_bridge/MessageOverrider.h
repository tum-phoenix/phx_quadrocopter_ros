/*
 * MessageOverrider.h
 *
 *  Created on: 15.07.2014
 *      Author: ic3
 */

#ifndef MESSAGEOVERRIDER_H_
#define MESSAGEOVERRIDER_H_

#include <phx_uart_bridge/MotorMessage.h>
#include <phx_uart_bridge/SensorMessage.h>

#include <ros/ros.h>

/**
 * Interface for classes that override the Motor- or SensorMessage before they get published.
 */
class MessageOverrider {

public:
	virtual ~MessageOverrider() { };

	/**
	 * Overrides the movement indicators with the kinect commands, if available
	 *
	 * @return true of data has been overridden
	 */
	virtual bool overrideSensor(phx_uart_bridge::SensorMessage & sensorMessage) const {
		return false;
	}

	/**
	 * Overrides the motor parameters with the values from the kinect
	 *
	 * @return true of data has been overridden
	 */
	virtual bool overrideMotor(phx_uart_bridge::MotorMessage & motorMessage) const {
		return false;
	}

};



#endif /* MESSAGEOVERRIDER_H_ */
