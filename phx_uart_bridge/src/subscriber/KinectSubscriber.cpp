/*
 * KinectSubscriber.cpp
 *
 *  Created on: 16.10.2013
 *      Author: ic3
 */

#include <phx_uart_bridge/MessageOverrider.h>
#include <phx_uart_bridge/subscriber/KinectSubscriber.h>

// TODO Remove motor subscriber, as it's only a workaround.

/**
 * This KinectSubscriber implementation subscribes to a ROS topic to receive the Kinect-data.
 */
KinectSubscriber::KinectSubscriber(ros::NodeHandle & nh) {

	isEnabled = false;

	std::string param;
	nh.param("/KinectBridge/topic/activate", param, std::string("/KinectBridge/activate"));
	nh.subscribe(param, 1, &KinectSubscriber::processActivateMessageCb, this);

	nh.param("/KinectBridge/topic/command", param, std::string("/KinectBridge/command"));
	nh.subscribe(param, 1, &KinectSubscriber::processSensorMessageCb, this);

	nh.param("/KinectBridge/topic/command/motor", param, std::string("/KinectBridge/command/motor"));
	nh.subscribe(param, 1, &KinectSubscriber::processMotorMessageCb, this);

	ROS_INFO("Initialized KinectSubscriber");
}

void KinectSubscriber::processSensorMessageCb(const phx_uart_bridge::SensorMessage& msg) {
	sensorMsg.radio0 = msg.radio0;
	sensorMsg.radio1 = msg.radio1;
	sensorMsg.radio2 = msg.radio2;
	sensorMsg.radio3 = msg.radio3;
}

void KinectSubscriber::processMotorMessageCb(const phx_uart_bridge::MotorMessage& msg) {
	motorMsg.motor0 = msg.motor0;
	motorMsg.motor1 = msg.motor1;
	motorMsg.motor2 = msg.motor2;
	motorMsg.motor3 = msg.motor3;
}

void KinectSubscriber::processActivateMessageCb(const std_msgs::String& msg) {
	isEnabled = msg.data.compare("true") == 0;
}

bool KinectSubscriber::overrideSensor(phx_uart_bridge::SensorMessage & sensorMessage) const {

	// Abort if no activation-command has been sent yet
	if ( !isEnabled )
        return false;

	sensorMessage.radio0 = sensorMsg.radio0;
	sensorMessage.radio1 = sensorMsg.radio1;
	sensorMessage.radio2 = sensorMsg.radio2;
	sensorMessage.radio3 = sensorMsg.radio3;

    return true;
}

bool KinectSubscriber::overrideMotor(phx_uart_bridge::MotorMessage & motorMessage) const {

	// Abort if override is disabled
	if ( !isEnabled )
        return false;

    motorMessage.motor0 = motorMsg.motor0;
    motorMessage.motor1 = motorMsg.motor1;
    motorMessage.motor2 = motorMsg.motor2;
    motorMessage.motor3 = motorMsg.motor3;

    return true;
}
