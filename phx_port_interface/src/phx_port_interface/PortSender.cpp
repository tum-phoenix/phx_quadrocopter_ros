/*
 * PortSender.cpp
 *
 *  Created on: 17.08.2013
 *      Author: ic3
 */

#include <phx_port_interface/PortSender.h>
#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>

PortSender::PortSender() :
	PortInteractor() {

}

PortSender::PortSender(const char* targetPort) :
		PortInteractor() {

	connect(targetPort);
}

void PortSender::sendMessage(void * msg, int length) {

	int len = sendto(socketfd, msg, length, 0, servinfo->ai_addr, servinfo->ai_addrlen);

	if (len < 0)
		ROS_ERROR("[PortSender] Failed to send message");
}
