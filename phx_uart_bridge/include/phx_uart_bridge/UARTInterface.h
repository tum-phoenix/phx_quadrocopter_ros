/*
 * UARTInterface.h
 *
 *  Created on: 17.10.2013
 *      Author: ic3
 */

#ifndef UARTINTERFACE_H_
#define UARTINTERFACE_H_

#include <phx_uart_bridge/MotorMessage.h>
#include <phx_uart_bridge/SensorMessage.h>
#include <phx_uart_bridge/UARTBridgeParams.h>

#include <ros/ros.h>
#include <fcntl.h>

enum AccessType {
	ReadOnly = O_RDONLY,
	ReadWrite = O_RDWR,
	WriteOnly = O_WRONLY
};

class UARTInterface {

public:

	UARTInterface();
	~UARTInterface();

	/**
	 * Writes the given string to UART
	 *
	 * @return false if there was an error
	 */
	bool writeString(const char * buffer, int size);

	/**
	 * @return next character received from UART
	 */
	char readChar();

	/**
	 * Reads a given number of characters from UART
	 *
	 * @return false if there was an error
	 */
	bool readString(char * buffer, int size);

	/*
	 * Populates uartReceivePackage with values from UART
	 */
	bool readUARTPackage(phx_uart_bridge::SensorMessage & sensorMessage);

	/*
	 * Configure the UART port
	 *
	 * @param r (optional) true -> may read from UART (default: true)
	 * @param w (optional) true -> may write to UART (default: true)
	 */
	int openUART(ros::NodeHandle & nh, AccessType t);

	/**
	 * Closes the file handle for the UART interface
	 */
	void closeUART();

private:

	int fd;
	unsigned char charBuffer[18];

	/*
	 * Waits for the start byte received from the UART port
	 */
	bool waitForStartByte();

	/*
	 * Reads a number from UART. singleByte is true if it's a 8-bit int, false if it's a 16-bit int.
	 *
	 * Returns false if reading from UART failed.
	 */
	bool readUARTNumber(short * number, bool singleByte);

	/**
	 * Harcoded workaround for open-issues
	 */
	void openUART_NDELAYFIX(const char * uartPath, int baudrate);

	/**
	 * Opens the specified UART port using the given flags
	 */
	int openUART(const char * path, int flags, int baudrate);

};


#endif /* UARTINTERFACE_H_ */
