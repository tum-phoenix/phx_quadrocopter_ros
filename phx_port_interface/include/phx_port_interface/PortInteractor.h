/*
 * PortInteractor.h
 *
 *  Created on: 17.08.2013
 *      Author: ic3
 */

#ifndef PORTINTERACTOR_H_
#define PORTINTERACTOR_H_

#include <netdb.h>
#include <ros/ros.h>

class PortInteractor {

public:
	virtual ~PortInteractor();

	/**
	 * Creates a socket for the specified port
	 */
	bool connect(const char * port);

protected:
	int socketfd;                         // socket file descriptor returned by socket()
	struct addrinfo hints;                // Configuration of the socket
	struct addrinfo *servinfo;            // will point to the results

	/**
	 * Evaluates the given status and prints an error message if applicable
	 */
	int evaluateStatus(int status) {
		if (status < 0)
			ROS_ERROR("PortInteractor Error: %s\n", gai_strerror(status));

		return status;
	}
};

#endif /* PORTINTERACTOR_H_ */
