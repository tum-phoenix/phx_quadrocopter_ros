/*
 * MotorMessageMerger.cpp
 *
 *  Created on: 15.07.2014
 *      Author: ic3
 */

#include <phx_uart_bridge/subscriber/KinectSubscriber.h>
#include <phx_uart_bridge/subscriber/WorkstationOverrideSubscriber.h>
#include <phx_uart_bridge/MotorMessageMerger.h>
#include <phx_uart_bridge/UARTBridgeParams.h>

MotorMessageMerger::MotorMessageMerger(ros::NodeHandle & nh) {

	publisher = nh.advertise<phx_uart_bridge::MotorMessage>(UARTBridgeParams::TOPIC_MOTOR, 1);

	// Create the message override instances, highest priority first.
	WorkstationOverrideSubscriber s3(nh);
	overriders.push_back(s3);

	KinectSubscriber s0(nh);
	overriders.push_back(s0);

	// Register the callback
	nh.subscribe(UARTBridgeParams::TOPIC_MOTOR_RAW, 1, &MotorMessageMerger::rawMotorMessageCb, this);
}

void MotorMessageMerger::rawMotorMessageCb(const phx_uart_bridge::MotorMessagePtr & raw) {

	// Update the local object
	msg.motor0 = raw->motor0;
	msg.motor1 = raw->motor1;
	msg.motor2 = raw->motor2;
	msg.motor3 = raw->motor3;

	// Iterate through all overridering instances. Abort once it has been overridden.
	for (std::vector<MessageOverrider>::const_iterator itr = overriders.begin(); itr != overriders.end(); ++itr )
		if (itr->overrideMotor(msg))
			break;

	// Eventually publish the (updated) message
	publisher.publish(msg);
}
