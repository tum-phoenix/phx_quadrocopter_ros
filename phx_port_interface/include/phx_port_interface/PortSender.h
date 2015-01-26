/*
 * PortSender.h
 *
 *  Created on: 17.08.2013
 *      Author: ic3
 */

#ifndef PORTSENDER_H_
#define PORTSENDER_H_

#define TUMPHOENIX_PORTSENDER_PORT_PARAM "sender_dst"

#include "PortInteractor.h"

class PortSender: public PortInteractor {
public:

	PortSender();
	PortSender(const char* targetPort);

	using PortInteractor::connect;

	/**
	 * Sends the provided message to the targeted destination
	 */
	void sendMessage(void * msg, int length);

};

#endif /* PORTSENDER_H_ */
