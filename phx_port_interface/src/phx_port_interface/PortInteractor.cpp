/*
 * PortInteractor.cpp
 *
 *  Created on: 17.08.2013
 *      Author: ic3
 */

#include <phx_port_interface/PortInteractor.h>
#include <sys/types.h>
#include <sys/socket.h>

PortInteractor::~PortInteractor() {
  close(socketfd);
  freeaddrinfo(servinfo); // free the linked-list
}

bool PortInteractor::connect(const char * port) {

	// Close socket if it has been opened before
	if ( socketfd >= 0 )
		close(socketfd);

	memset(&hints, 0, sizeof hints);      // make sure the struct is empty
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;       // UDP stream sockets
	hints.ai_flags = AI_PASSIVE;

	// servinfo now points to a linked list of 1 or more struct addrinfos
	if ( evaluateStatus(getaddrinfo(NULL, port, &hints, &servinfo)) < 0 )
		return false;

	socketfd = evaluateStatus(socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol));
	return socketfd > 0;
}
