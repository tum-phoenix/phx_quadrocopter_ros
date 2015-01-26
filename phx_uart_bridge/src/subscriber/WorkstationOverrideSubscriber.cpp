/*
 * WorkstationOverrideSubscriber.cpp
 *
 *  Created on: 31.10.2013
 *      Author: ic3
 */
#include "phx_uart_bridge/subscriber/WorkstationOverrideSubscriber.h"

#define WO_DEBUG	false
#define WO_TRACE	false

WorkstationOverrideSubscriber::WorkstationOverrideSubscriber(ros::NodeHandle& nh) {

	isMotorActive = false;
	isSensorActive = false;
	std::string activationTopic;
	nh.param("workstationOverrideActivationTopic", activationTopic, std::string("/workstationOverride/activate"));
	activationSubscriber = nh.subscribe(activationTopic, 1, &WorkstationOverrideSubscriber::processActivationMessageCb, this);

	std::string sensorTopic;
	nh.param("workstationOverrideSensorTopic", sensorTopic, std::string("/workstationOverride/sensor"));
	sensorSubscriber = nh.subscribe(sensorTopic, 1, &WorkstationOverrideSubscriber::processSensorMessageCb, this);

	std::string motorTopic;
	nh.param("workstationOverrideTopic", motorTopic, std::string("/workstationOverride/motor"));
	motorSubscriber = nh.subscribe(motorTopic, 1, &WorkstationOverrideSubscriber::processMotorMessageCb, this);

	ROS_INFO("Initialized WorkstationOverrideSubscriber");
}

bool WorkstationOverrideSubscriber::overrideSensor(phx_uart_bridge::SensorMessage & msg) const {
	if ( !isSensorActive )
        return false;

	msg.accX = sensorMessage.accX;
	msg.accY = sensorMessage.accY;
	msg.accZ = sensorMessage.accZ;
	msg.angleSpeedX = sensorMessage.angleSpeedX;
	msg.angleSpeedY = sensorMessage.angleSpeedY;
	msg.angleSpeedZ = sensorMessage.angleSpeedZ;
	msg.battery = sensorMessage.battery;
	msg.radio0 = sensorMessage.radio0;
	msg.radio1 = sensorMessage.radio1;
	msg.radio2 = sensorMessage.radio2;
	msg.radio3 = sensorMessage.radio3;
    return true;
}

bool WorkstationOverrideSubscriber::overrideMotor(phx_uart_bridge::MotorMessage & msg) const {
#if WO_TRACE
	if ( isMotorActive )
		ROS_INFO("MotorOverride: %4d | %4d | %4d | %4d", motorMessage.motor0, motorMessage.motor1, motorMessage.motor2, motorMessage.motor3);
	else
		ROS_INFO("MotorOverride: Disabled");
#endif

	if ( !isMotorActive )
        return false;

	msg.motor0 = motorMessage.motor0;
	msg.motor1 = motorMessage.motor1;
	msg.motor2 = motorMessage.motor2;
	msg.motor3 = motorMessage.motor3;
    return true;
}

void WorkstationOverrideSubscriber::processSensorMessageCb(const phx_uart_bridge::SensorMessage& msg) {
#if WO_DEBUG
	ROS_INFO("Received SensorMessage");
#endif

	sensorMessage.accX = msg.accX;
	sensorMessage.accY = msg.accY;
	sensorMessage.accZ = msg.accZ;
	sensorMessage.angleSpeedX = msg.angleSpeedX;
	sensorMessage.angleSpeedY = msg.angleSpeedY;
	sensorMessage.angleSpeedZ = msg.angleSpeedZ;
	sensorMessage.battery = msg.battery;
	sensorMessage.radio0 = msg.radio0;
	sensorMessage.radio1 = msg.radio1;
	sensorMessage.radio2 = msg.radio2;
	sensorMessage.radio3 = msg.radio3;
}

void WorkstationOverrideSubscriber::processMotorMessageCb(const phx_uart_bridge::MotorMessage& msg) {
#if WO_DEBUG
	ROS_INFO("Received MotorMessage: %4d | %4d | %4d | %4d", msg.motor0, msg.motor1, msg.motor2, msg.motor3);
#endif

	motorMessage.motor0 = msg.motor0;
	motorMessage.motor1 = msg.motor1;
	motorMessage.motor2 = msg.motor2;
	motorMessage.motor3 = msg.motor3;
}

/**
 * Returns true if the flag at the specified index indicates an active state
 */
inline bool isActive(const std_msgs::String& msg, int idx) {
	return msg.data.data()[idx] == '1';
}

void WorkstationOverrideSubscriber::processActivationMessageCb(const std_msgs::String& msg) {
	isMotorActive = isActive(msg, 0);
	isSensorActive = isActive(msg, 1);

#if WO_DEBUG
	ROS_INFO("Active Motor data: %d", isActive(msg, 0));
#endif
}
