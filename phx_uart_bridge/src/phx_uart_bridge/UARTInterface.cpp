/*
 * UARTInterface.cpp
 *
 *  Created on: 17.10.2013
 *      Author: ic3
 */

#include <phx_uart_bridge/UARTInterface.h>

#include <termios.h>

UARTInterface::UARTInterface() {
	fd = -1;
}

UARTInterface::~UARTInterface() {
	closeUART();
}

void UARTInterface::closeUART() {
	if (fd < 0)
		return;

    close(fd);
    fd = -1;
}

void UARTInterface::openUART_NDELAYFIX(const char * uartPath, int baudrate) {
	ROS_INFO("Opening UART port (O_NDELAY FIX): %s", uartPath);
	openUART(uartPath, ReadWrite | O_NOCTTY | O_NDELAY, baudrate);

	// Reading a couple of packages should do the trick
	for ( int i = 0; i < 100; i++ )
		read(fd, charBuffer, 1);

	closeUART();
}

int UARTInterface::openUART(ros::NodeHandle & nh, AccessType t) {

	// Retrieve the parameters
	std::string path;
	nh.param(UARTBridgeParams::PATH, path, UARTBridgeParams::PATH_DEFAULT);

	int baudrate = 0;
	nh.getParam(UARTBridgeParams::BAUDRATE, baudrate);
	switch (baudrate) {
		case 19200:
			baudrate = 0 | B19200;
			break;
		case 38400:
			baudrate = 0 | B38400;
			break;
		case 57600:
			baudrate = 0 | B57600;
			break;
		case 115200:
		default:
			baudrate = 0 | B115200;
			break;
	}

	// Determine the access flags (O_NOCTTY is default)
	int flags = O_NOCTTY | t;

	// Open the UART Port
	ROS_INFO("Opening UART: %s", path.c_str());

	// Harcoded workaround for open-issues
	// TODO WHY IS THIS HAPPENING TO ME
	openUART_NDELAYFIX(path.c_str(), baudrate);
	openUART(path.c_str(), flags, baudrate);

	return fd;
}

int UARTInterface::openUART(const char * path, int flags, int baudrate) {

	// Open the UART Port
	while (ros::ok())
		if ((fd = open(path, flags)) < 0)
			ROS_ERROR("Opening UART: Failed: open(%s, %d)", path, flags);
		else
			break;

	// Configure the UART port
	struct termios term_attr;
	if ( tcgetattr(fd, &term_attr) != 0 )
		ROS_ERROR("Opening UART: Failed: tcgetattr()");

	term_attr.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
	term_attr.c_iflag = 0;
	term_attr.c_oflag = 0;
	term_attr.c_lflag = 0;

	// VTIME: 10th of seconds timeout for next character until read() returns
	term_attr.c_cc[VTIME] = 1;
	// VMIN: # chars required for read() to return, overrides read(fd, buffer, n_bytes)
	term_attr.c_cc[VMIN] = 17;

	if (tcsetattr(fd, TCSAFLUSH, &term_attr) != 0)
		ROS_ERROR("Opening UART: Failed: tcsetattr()");

	ROS_INFO("Opening UART: Done");
	return fd;
}

// --- UART Read / Write operations -------------------------------------------

bool UARTInterface::writeString(const char * buffer, int size) {
	return write(fd, buffer, size) != size;
}

char UARTInterface::readChar() {
	char c[] = "\0";
	return (readString(&c[0], 1)) ? c[0] : '\0';
}

bool UARTInterface::readString(char * buffer, int size) {
	return ( read(fd, buffer, size) == -1 );
}
