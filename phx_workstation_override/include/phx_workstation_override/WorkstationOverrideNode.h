/*
 * WorkstationOverrideNode.h
 *
 *  Created on: 06.11.2013
 *      Author: ic3
 */

#ifndef WORKSTATIONOVERRIDENODE_H_
#define WORKSTATIONOVERRIDENODE_H_

#include <phx_uart_bridge/MotorMessage.h>
#include <phx_uart_bridge/SensorMessage.h>
#include <phx_workstation_override/WorkstationOverrideInstruction.h>
#include <phx_workstation_override/InitialMotorTest.h>
#include <phx_workstation_override/MarvelousSinger.h>
#include <phx_workstation_override/UserMotorOverride.h>
#include <phx_workstation_override/ManualMotorControl.h>
#include <phx_workstation_override/EmergencyStopOverride.h>
#include <ros/ros.h>

struct Colors {
	std::string Default, Selected;
	Colors() : Default("\e[0m"), Selected("\e[47;30m") { }
} colors;

struct AvailableOverrides {
	struct AvailableOverride {
		std::string label;
		int id;

		AvailableOverride(int id, std::string label) {
			this->label = label;
			this->id = id;
		}
	};

	AvailableOverride emergencyOverride;
	AvailableOverride initialMotorTest;
	AvailableOverride manualMotorOverride;
	AvailableOverride marvelousSinger;
	AvailableOverride userMotorOverride;
	AvailableOverride exit;

	AvailableOverrides() :
		emergencyOverride	(0, "Emergency Override"),
		initialMotorTest	(1, "Initial Motor Test"),
		manualMotorOverride	(2, "Manual Motor Override"),
		marvelousSinger		(3, "Marvelous Singer"),
		userMotorOverride	(4, "User Motor Override"),
		exit				(5, "Exit") {
	}
} availableOverrides;

/**
 * Returns the id of one of the available overrides
 */
int getUserChoice();

/**
 * Prints all available overrides
 */
void printChoices(int & activeChoice);

/**
 * Prints the given override choice
 */
void printChoice(AvailableOverrides::AvailableOverride & label, bool active);

// --- linux_getch ------------------------------------------------------------
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#define KB_UP		65
#define KB_DOWN		66
#define KB_ESC		27
#define KB_ENTER	10
#define END_FILE_CHARACTER 0x04  /* ctrl-d is unix-style eof input key*/

int linux_getch(void);

#endif /* WORKSTATIONOVERRIDENODE_H_ */
