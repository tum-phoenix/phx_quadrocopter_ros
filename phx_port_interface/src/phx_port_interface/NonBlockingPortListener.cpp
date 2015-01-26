/*
 * NonBlockingPortListener.cpp
 *
 *  Created on: 19.10.2013
 *      Author: ic3
 */

#include <phx_port_interface/NonBlockingPortListener.h>

NonBlockingPortListener::~NonBlockingPortListener() {
	if (socketfd > 0)
		close(socketfd);
}

int NonBlockingPortListener::open(short port) {

	// Open the socket
	socketfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketfd < 0) {
		ROS_ERROR("[NonBlockingPortListener] Failed to register socket");
		return socketfd;
	}

	int flags = fcntl(socketfd, F_GETFL);
	flags |= O_NONBLOCK;

	// setting up destination addres
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);

	// Bind the socket to the specified port
	if (bind(socketfd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		ROS_ERROR("[NonBlockingPortListener] Failed to bind socket on port %d", port);
		return -1;
	}

	addrlen = sizeof(addr);
	fcntl(socketfd, F_SETFL, flags);

	ROS_INFO("[NonBlockingPortListener] Opened non-blocking listener for port %d", port);
	return socketfd;
}

int NonBlockingPortListener::readNext(void * msgbuf) {
	return recvfrom(socketfd, msgbuf, sizeof(msgbuf), 0, (struct sockaddr *) &addr, (socklen_t*) &addrlen);
}
