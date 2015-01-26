/*
 * NonBlockingPortListener.h
 *
 *  Created on: 19.10.2013
 *      Author: ic3
 */

#ifndef NONBLOCKINGPORTLISTENER_H_
#define NONBLOCKINGPORTLISTENER_H_

#include <ros/ros.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>

class NonBlockingPortListener {

public:

	virtual ~NonBlockingPortListener();

	/**
	 * Reads the next package into the buffer
	 *
	 * @return length of the message or -1, if no package has been received
	 */
	int readNext(void * buffer);

	/**
	 * Opens a socket and binds it to the given port
	 */
	int open(short port);

private:

	int socketfd;
	int addrlen;
	struct sockaddr_in addr;

};


#endif /* NONBLOCKINGPORTLISTENER_H_ */
