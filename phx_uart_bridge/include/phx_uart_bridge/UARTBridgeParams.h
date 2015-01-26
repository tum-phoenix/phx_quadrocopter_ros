/*
 * UARTInterfaceParams.h
 *
 *  Created on: 13.07.2014
 *      Author: ic3
 */

#ifndef UARTINTERFACEPARAMS_H_
#define UARTINTERFACEPARAMS_H_

#include <ros/ros.h>

// Print some debug messages
#define UARTINTERFACE_DEBUG					false

// Print the message received from UART
#define UARTINTERFACE_RECEIVE				false

// Below are working UART_SPEEDS on pandaboard
//#define BAUD_RATE	B19200
//#define BAUD_RATE	B38400
#define BAUD_RATE	B57600
//#define BAUD_RATE	B115200

struct UARTBridgeParams {

	static const std::string BAUDRATE;
	static const std::string PATH;
	static const std::string PATH_DEFAULT;

	static const std::string TOPIC_MOTOR;
	static const std::string TOPIC_MOTOR_RAW;
	static const std::string TOPIC_SENSOR;
	static const std::string TOPIC_SENSOR_RAW;

};



#endif /* UARTINTERFACEPARAMS_H_ */
