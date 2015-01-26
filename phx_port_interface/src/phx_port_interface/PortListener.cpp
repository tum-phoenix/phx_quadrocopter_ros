/*
 * PortListener.cpp
 *
 *  Created on: 17.08.2013
 *      Author: ic3
 */

#include <phx_port_interface/PortListener.h>

#include <sys/socket.h>

PortListener::PortListener(const char * port) :
		PortInteractor() {

	connect(port);
}

PortListener::PortListener() :
		PortInteractor() {

	their_addr_len = 0;
}

bool PortListener::connect(const char * port, int timeout) {

	// Open socket using super implementation
	PortInteractor::connect(port);

	// Bind this process to the port
	int s = bind(socketfd, servinfo->ai_addr, servinfo->ai_addrlen);
	if (evaluateStatus(s) < 0)
		return false;

	their_addr_len = sizeof their_addr;

	// Return now if no timeout has been specified
	if ( timeout <= 0 )
		return true;

	struct timeval tv;
	tv.tv_sec = timeout;
//	tv.tv_usec = 100000;
	if (setsockopt(socketfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
		return false;

	else
		return true;
}

char * PortListener::getNextPackage() {
	int len = readNext(buffer);
	buffer[len] = '\n';
	return buffer;
}

int PortListener::readNext(void * buffer) {
	return recvfrom(socketfd, buffer, MAXBUFLEN - 1, 0, (struct sockaddr *) &their_addr, &their_addr_len);
}
