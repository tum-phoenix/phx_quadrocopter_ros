/*
 * EmergencyStopOverride.cpp
 *
 *  Created on: 04.11.2013
 *      Author: ic3
 */

#include <phx_workstation_override/EmergencyStopOverride.h>

std::string EmergencyStopOverride::ReleaseCmd("release");

EmergencyStopOverride::EmergencyStopOverride(WOIPublisher & woiPublisher) :
	WorkstationOverrideInstruction(woiPublisher) {

	msg.motor0 = 0;
	msg.motor1 = 0;
	msg.motor2 = 0;
	msg.motor3 = 0;
}

void EmergencyStopOverride::runOverride() {

	// Halt all motors
	setOverrideFlags(true, false);
	motorPub.publish(msg);

	std::cout << "Emergency stop issued." << std::endl;

	// Wait for user input to disable the override
	std::string line;
	while (true) {
		std::cout << "Enter '" << ReleaseCmd << "' to disable the override" << std::endl;
		std::cout << ">> ";
		std::getline(std::cin, line);
		if ( ReleaseCmd.compare(line) == 0 )
			break;
		else
			std::cout << "Wrong input, maintaining lock." << std::endl;
	}

	// Eventually, disable motor override again
	setOverrideFlags(false, false);
	std::cout << "Emergency stop revoked." << std::endl;
}
