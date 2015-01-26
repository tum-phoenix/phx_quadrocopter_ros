/*
 * InitialMotorTest.h
 *
 *  Created on: 01.11.2013
 *      Author: ic3
 */

#ifndef INITIALMOTORTEST_H_
#define INITIALMOTORTEST_H_

#include <phx_workstation_override/WorkstationOverrideInstruction.h>
#include <phx_uart_bridge/MotorMessage.h>

#include <ros/ros.h>

/**
 * Overrides the motor data and performs a basic motor check.
 */
class InitialMotorTest : public WorkstationOverrideInstruction {

public:

	InitialMotorTest(WOIPublisher & woiPublisher);

	void runOverride();

private:

	phx_uart_bridge::MotorMessage msg;

	int lowVal;
	int highVal;

	/**
	 * Tests all motors, one after another. Returns false if the test has been aborted.
	 */
	bool singleMotorCircleTest(ros::Duration testTime);

};


#endif /* INITIALMOTORTEST_H_ */
