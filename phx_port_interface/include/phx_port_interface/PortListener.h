/*
 * PortInteractor.h
 *
 *  Created on: 17.08.2013
 *      Author: ic3
 */

#ifndef PORTLISTENER_H_
#define PORTLISTENER_H_

#include "PortInteractor.h"

#define MAXBUFLEN 100
#define TUMPHOENIX_PORTLISTENER_PORT_PARAM "listener_src"

class PortListener: public PortInteractor {
public:

	PortListener();
	PortListener(const char* port);

	/**
	 * Waits for the next package and returns the message
	 */
	char* getNextPackage();

	/**
	 * Reads the next package into the buffer
	 *
	 * @return length of the message
	 */
	int readNext(void * buffer);

	/**
	 * Opens a socket on the specified port, ready to read incoming messages.
	 *
	 * @param timeout unit: seconds
	 */
	bool connect(const char * port, int timeout = 1);

protected:

	struct sockaddr_storage their_addr;
	socklen_t their_addr_len;
	char buffer[MAXBUFLEN];
};

#endif /* PORTLISTENER_H_ */
