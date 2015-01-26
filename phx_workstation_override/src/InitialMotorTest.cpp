/*
 * InitialMotorTest.cpp
 *
 *  Created on: 01.11.2013
 *      Author: ic3
 */

#include <phx_workstation_override/InitialMotorTest.h>

InitialMotorTest::InitialMotorTest(WOIPublisher & woiPublisher) :
	WorkstationOverrideInstruction(woiPublisher) {

	lowVal = 0;
	highVal = 1;
}

void InitialMotorTest::runOverride() {

	ROS_INFO("Starting InitialMotorTest procedure");

	setOverrideFlags(true, false);

	// Run multiple tests with different time spans
	if ( singleMotorCircleTest(ros::Duration(1)) &&
			singleMotorCircleTest(ros::Duration(.7)) &&
			singleMotorCircleTest(ros::Duration(.5)) &&
			singleMotorCircleTest(ros::Duration(.3)) )
		ROS_INFO("All tests completed");

	// Halt all motors
	msg.motor0 = 0;
	msg.motor1 = 0;
	msg.motor2 = 0;
	msg.motor3 = 0;
	motorPub.publish(msg);

	// Eventually, disable motor override again
	setOverrideFlags(false, false);

	ROS_INFO("Test procedure done");
}

bool InitialMotorTest::singleMotorCircleTest(ros::Duration testTime) {
	if ( !ros::ok() )
		return false;

	ROS_INFO("Circular Single-Motor Test: %d.%d", testTime.sec, testTime.nsec);

	ROS_INFO("Testing motor0");
	msg.motor0 = 4500;
	msg.motor1 = 0;
	msg.motor2 = 0;
	msg.motor3 = 0;
	motorPub.publish(msg);
	testTime.sleep();

	if ( !ros::ok() )
		return false;

	ROS_INFO("Testing motor1");
	msg.motor0 = 0;
	msg.motor1 = 4500;
	motorPub.publish(msg);
	testTime.sleep();

	if ( !ros::ok() )
		return false;

	ROS_INFO("Testing motor2");
	msg.motor1 = 0;
	msg.motor2 = 4500;
	motorPub.publish(msg);
	testTime.sleep();

	if ( !ros::ok() )
		return false;

	ROS_INFO("Testing motor3");
	msg.motor2 = 0;
	msg.motor3 = 4500;
	motorPub.publish(msg);
	testTime.sleep();

	return true;
}
