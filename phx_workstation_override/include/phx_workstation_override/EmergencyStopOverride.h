/*
 * EmergencyStopOverride.h
 *
 *  Created on: 04.11.2013
 *      Author: ic3
 */

#ifndef EMERGENCYSTOPOVERRIDE_H_
#define EMERGENCYSTOPOVERRIDE_H_

#include <phx_workstation_override/WorkstationOverrideInstruction.h>
#include <phx_uart_bridge/MotorMessage.h>

class EmergencyStopOverride : public WorkstationOverrideInstruction {

public:

	static std::string ReleaseCmd;

	EmergencyStopOverride(WOIPublisher & woiPublisher);

	void runOverride();

private:

	phx_uart_bridge::MotorMessage msg;

};


#endif /* EMERGENCYSTOPOVERRIDE_H_ */
