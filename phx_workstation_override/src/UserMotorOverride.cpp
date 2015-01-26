/*
 * UserMotorOverride.cpp
 *
 *  Created on: 03.11.2013
 *      Author: ic3
 */

#include <phx_workstation_override/UserMotorOverride.h>

std::string UserMotorOverride::InputPrefix = "MotorVal: ";
std::string UserMotorOverride::CMD_Exit = "exit";
std::string UserMotorOverride::CMD_On = "on";
std::string UserMotorOverride::CMD_Off = "off";
std::string UserMotorOverride::CMD_Higher = "w";
std::string UserMotorOverride::CMD_Lower = "s";
std::string UserMotorOverride::CMD_Down = "a";

UserMotorOverride::UserMotorOverride(WOIPublisher & woiPublisher) :
	WorkstationOverrideInstruction(woiPublisher) {

}

void UserMotorOverride::runOverride() {
    ROS_INFO("Startig UserMotorOverride. Enter 'exit' to return to menu.");
	setOverrideFlags(true, false);

	// Process user input until exit-command or ros::ok() is false
	std::cout << InputPrefix;
	for (std::string line; std::getline(std::cin, line) && ros::ok(); ) {

		if ( CMD_Exit.compare(line) == 0 )
			break;

		else if ( CMD_On.compare(line) == 0 )
			enableOverride();

		else if ( CMD_Off.compare(line) == 0 )
			disableOverride();

		else if ( CMD_Higher.compare(line) == 0 )
			sendMotorData(msg.motor0 + 100);

		else if ( CMD_Lower.compare(line) == 0 )
			sendMotorData(msg.motor0 - 100);

		else if ( CMD_Down.compare(line) == 0 )
			sendMotorData(500);

		else
			sendMotorData(line);

        std::cout << InputPrefix;
	}

	ROS_INFO("Quitting UserMotorOverride");
	setOverrideFlags(false, false);
}

void UserMotorOverride::enableOverride() {
	std::cout << "Activating override ... ";
	setOverrideFlags(true, true);
	std::cout << "Done" << std::endl;
}

void UserMotorOverride::disableOverride() {
	std::cout << "Disabling override ... ";
	setOverrideFlags(true, true);
	std::cout << "Done" << std::endl;
}

void UserMotorOverride::sendMotorData(std::string & line) {

	// Convert the input to a number and publish it
	int val = atoi(line.c_str());
	sendMotorData(val);
}

void UserMotorOverride::sendMotorData(int val) {
	if ( val < 1800 ) {
		std::cout << "Value '" << val << "' too low, defaulting to 1800" << std::endl;
		val = 1800;
	}

	else if ( val > 3600 ) {
		std::cout << "Value '" << val << "' too high, defaulting to 3600" << std::endl;
		val = 3600;
	}

	msg.motor0 = val;
	msg.motor1 = val;
	msg.motor2 = val;
	msg.motor3 = val;
	motorPub.publish(msg);

	std::cout << "MotorOverride: " << val << std::endl;
}
