/*
 * UserMotorOverride.h
 *
 *  Created on: 03.11.2013
 *      Author: ic3
 */

#ifndef USERMOTOROVERRIDE_H_
#define USERMOTOROVERRIDE_H_

#include <phx_workstation_override/WorkstationOverrideInstruction.h>
#include <phx_uart_bridge/MotorMessage.h>

#include <ros/ros.h>

/**
 * Overrides the motor data and sets it to values specified by the user (StdIn).
 */
class UserMotorOverride : public WorkstationOverrideInstruction {

public:

	static std::string InputPrefix, CMD_Exit, CMD_On, CMD_Off, CMD_Higher, CMD_Lower, CMD_Down;

	UserMotorOverride(WOIPublisher & woiPublisher);

	void runOverride();

private:

	phx_uart_bridge::MotorMessage msg;

	void enableOverride();
	void disableOverride();
	void sendMotorData(std::string & line);
	void sendMotorData(int val);

};



#endif /* USERMOTOROVERRIDE_H_ */
