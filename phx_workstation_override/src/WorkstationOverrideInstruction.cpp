/*
 * WorkstationOverrideInstruction.cpp
 *
 *  Created on: 01.11.2013
 *      Author: ic3
 */

#include <phx_workstation_override/WorkstationOverrideInstruction.h>

WorkstationOverrideInstruction::WorkstationOverrideInstruction(ros::Publisher & activatePub, ros::Publisher & motorPub, ros::Publisher & sensorPub)
	: activatePub(activatePub), motorPub(motorPub), sensorPub(sensorPub) {

}

WorkstationOverrideInstruction::WorkstationOverrideInstruction(WOIPublisher & woiPublisher)
	: activatePub(woiPublisher.activatePublisher), motorPub(woiPublisher.motorPublisher), sensorPub(woiPublisher.sensorPublisher) {

}


WorkstationOverrideInstruction::~WorkstationOverrideInstruction() {

}

void WorkstationOverrideInstruction::setOverrideFlags(bool motor, bool sensor) {

	if ( motor || sensor )
		ros::Duration(.5).sleep();

	std_msgs::String msg;
	if ( motor && sensor )
		msg.data = "11";

	else if ( motor )
		msg.data = "10";

	else if ( sensor )
		msg.data = "01";

	else
		msg.data = "00";

	if ( !(motor || sensor) )
		ros::Duration(.5).sleep();

	activatePub.publish(msg);
}

