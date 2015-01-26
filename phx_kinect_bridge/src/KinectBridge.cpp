/*
 * KinectBridge.cpp
 *
 * Its task is to receive the data from the computer running the Kinect and publish it to ROS.
 *
 * Target is the UARTBridgeNode on the same machine, which is why
 * we can use certain techniques to improve transfer speed.
 *
 *  Created on: 16.10.2013
 *      Author: ic3
 */

#include <phx_kinect_bridge/KinectBridge.h>

#define KINECT_DEBUG true
#define KINECT_MOTOR_DEFAULT	2300
#define KINECT_MOTOR_HIGH		2800
#define KINECT_MOTOR_LOW		2100

KinectBridge::KinectBridge(ros::NodeHandle & nh) {

	// ROS Publisher for Kinect
	activatePublisher = nh.advertise<std_msgs::String>(KinectParams::TOPIC_ACTIVATE, 1);
	sensorPublisher = nh.advertise<phx_uart_bridge::SensorMessage>(KinectParams::TOPIC_COMMAND, 1);
	motorPublisher = nh.advertise<phx_uart_bridge::MotorMessage>(KinectParams::TOPIC_COMMAND_MOTOR, 1);

	// PortListener for the Kinect messages
	std::string param;
	nh.param(KinectParams::PORT, param, std::string("31339"));
	portListener.connect(param.c_str(), 1);

	ROS_INFO("Initialized KinectBridge");
}

void KinectBridge::startBridge() {
	while ( ros::ok() )
		publishMessage();
}

void KinectBridge::getNextMessage() {

	// Read next package and abort if unable to receive
	int nbytes = portListener.readNext(buffer);
	if ( nbytes < 0 ) {
		kinectMessage.command = KINECT_NONE;
        kinectMessage.value = -1;
		return;
	}
	else
        buffer[nbytes] = '\0';

    // Split upon blanks: "CMD VAL"
    std::string val(buffer);
    std::vector<std::string> parts = split(val, ' ');

    kinectMessage.command = atoi(parts[0].c_str());
    kinectMessage.value = atoi(parts[1].c_str());

#if KINECT_DEBUG
	ROS_INFO("[KINECT] Received %s", val.c_str());
#endif
}

void KinectBridge::publishMessage() {

	getNextMessage();

	// Resolve the Kinect command to a movement indication
    // TODO Implement relation between angle and radio value
	switch (this->kinectMessage.command) {

		case KINECT_UP:
            publishMessage(0, 0, 110, 0);
            publishMotorMessage(KINECT_MOTOR_HIGH, KINECT_MOTOR_HIGH, KINECT_MOTOR_HIGH, KINECT_MOTOR_HIGH);
            return;

		case KINECT_DOWN:
            publishMessage(0, 0, 80, 0);
            publishMotorMessage(KINECT_MOTOR_LOW, KINECT_MOTOR_LOW, KINECT_MOTOR_LOW, KINECT_MOTOR_LOW);
            return;

		case KINECT_ACCELERATE:
            publishMessage(0, 0, 0, 110);
            publishMotorMessage(KINECT_MOTOR_LOW, KINECT_MOTOR_DEFAULT, KINECT_MOTOR_HIGH, KINECT_MOTOR_DEFAULT);
            return;

        case KINECT_BACKWARDS:
            publishMessage(0, 0, 0, 80);
            publishMotorMessage(KINECT_MOTOR_HIGH, KINECT_MOTOR_DEFAULT, KINECT_MOTOR_LOW, KINECT_MOTOR_DEFAULT);
            return;

		case KINECT_LEFT:
            publishMessage(0, 110, 0, 0);
            publishMotorMessage(KINECT_MOTOR_DEFAULT, KINECT_MOTOR_LOW, KINECT_MOTOR_DEFAULT, KINECT_MOTOR_HIGH);
            return;

        case KINECT_RIGHT:
            publishMessage(0, 80, 0, 0);
            publishMotorMessage(KINECT_MOTOR_DEFAULT, KINECT_MOTOR_HIGH, KINECT_MOTOR_DEFAULT, KINECT_MOTOR_LOW);
            return;

			// Enable
		case KINECT_ENABLE:
			activateMessage.data = "true";
			activatePublisher.publish(activateMessage);
            return;

			// Disable
		case KINECT_DISABLE:
			activateMessage.data = "false";
			activatePublisher.publish(activateMessage);
			return;
	}
}


// --- Convenience methods ----------------------------------------------------

void KinectBridge::publishMessage(int16_t radio0, int16_t radio1, int16_t radio2, int16_t radio3) {
    sensorMessage.radio0 = radio0;
    sensorMessage.radio1 = radio1;
    sensorMessage.radio2 = radio2;
    sensorMessage.radio3 = radio3;

    sensorPublisher.publish(sensorMessage);
}

void KinectBridge::publishMotorMessage(int16_t motor0, int16_t motor1, int16_t motor2, int16_t motor3) {
	motorMessage.motor0 = motor0;
	motorMessage.motor1 = motor1;
	motorMessage.motor2 = motor2;
	motorMessage.motor3 = motor3;

	motorPublisher.publish(motorMessage);
}

std::vector<std::string> & KinectBridge::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
        elems.push_back(item);

    return elems;
}

std::vector<std::string> KinectBridge::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);

    return elems;
}
