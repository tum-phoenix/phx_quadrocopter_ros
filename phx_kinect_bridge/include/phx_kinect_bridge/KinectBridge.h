/*
 * KinectBridge.h
 *
 *  Created on: 07.11.2013
 *      Author: ic3
 */

#ifndef KINECTBRIDGE_H_
#define KINECTBRIDGE_H_

#define KINECT_NONE			0
#define KINECT_UP			1
#define KINECT_DOWN			2
#define KINECT_ACCELERATE	3
#define KINECT_BACKWARDS	4
#define KINECT_ROTZ			5
#define KINECT_ROTY			6
#define	KINECT_ROTX			7
#define KINECT_STOP			8
#define	KINECT_LEFT			9
#define	KINECT_RIGHT		10
#define	KINECT_ENABLE		11
#define KINECT_DISABLE		KINECT_STOP

#include <phx_port_interface/PortListener.h>
#include <phx_uart_bridge/MotorMessage.h>
#include <phx_uart_bridge/SensorMessage.h>
#include <phx_kinect_bridge/KinectParams.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

class KinectBridge {

public:

	KinectBridge(ros::NodeHandle & nh);

	/**
	 * Starts to publish all incoming messages to the ROS world
	 */
	void startBridge();

private:

    // --- UDP Interface ------------------------------------------------------
	struct KinectMessage {
		int command;
		int value;

		KinectMessage() {
			command = KINECT_NONE;
			value = 0;
		}
	} kinectMessage;

	PortListener portListener;
	char buffer[100];

    // --- ROS Interface ------------------------------------------------------

	phx_uart_bridge::SensorMessage sensorMessage;
	ros::Publisher sensorPublisher;
	phx_uart_bridge::MotorMessage motorMessage;
	ros::Publisher motorPublisher;
	std_msgs::String activateMessage;
	ros::Publisher activatePublisher;

	/**
	 * Returns as soon as a new message has been retrieved
	 */
	void getNextMessage();

	/**
	 * Publishes the next incoming message to the ROS world
	 */
	void publishMessage();

    // --- Misc. methods ------------------------------------------------------

    /**
     * Updates the SensorMessage with the specified values and publishes it via ROS
     */
    void publishMessage(int16_t radio0, int16_t radio1, int16_t radio2, int16_t radio3);

    /**
     * Updates the MotorMessage with the specified values and publishes it via ROS
     */
    void publishMotorMessage(int16_t motor0, int16_t motor1, int16_t motor2, int16_t motor3);


    /**
     * Splits the given string upon specified delimiters
     */
    std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);

    /**
     * Convenience method for split
     */
    std::vector<std::string> split(const std::string &s, char delim);

};

#endif /* KINECTBRIDGE_H_ */
