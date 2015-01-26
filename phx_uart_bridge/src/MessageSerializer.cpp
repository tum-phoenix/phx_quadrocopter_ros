/*
 * MessageSerializer.cpp
 *
 *  Created on: 15.07.2014
 *      Author: ic3
 */

#include <phx_uart_bridge/MessageSerializer.h>

int MessageSerializer::serialize(phx_uart_bridge::MotorMessage & msg, char buffer[]) {
	buffer[0] = 'U';
	buffer[1] = msg.motor0 >> 8;
	buffer[2] = msg.motor0;
	buffer[3] = msg.motor1 >> 8;
	buffer[4] = msg.motor1;
	buffer[5] = msg.motor2 >> 8;
	buffer[6] = msg.motor2;
	buffer[7] = msg.motor3 >> 8;
	buffer[8] = msg.motor3;

	return 9;
}

void MessageSerializer::deserialize(phx_uart_bridge::SensorMessagePtr & msg, const char buffer[]) {
	msg->radio0 = buffer[0];
	msg->radio1 = buffer[1];
	msg->radio2 = buffer[2];
	msg->radio3 = buffer[3];
	msg->accX = buffer[4] | (buffer[5] << 8);
	msg->accY = buffer[6] | (buffer[7] << 8);
	msg->accZ = buffer[8]| (buffer[9] << 8);
	msg->angleSpeedX = buffer[10]| (buffer[11] << 8);
	msg->angleSpeedY = buffer[12]| (buffer[13] << 8);
	msg->angleSpeedZ = buffer[14]| (buffer[15] << 8);
	msg->battery = buffer[16];
}
