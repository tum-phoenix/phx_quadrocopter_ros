/*
 * ManualMotorControl.h
 *
 *  Created on: 01.11.2013
 *      Author: ic3
 */

#ifndef MANUALMOTORCONTROL_H_
#define MANUALMOTORCONTROL_H_

#include <phx_workstation_override/WorkstationOverrideInstruction.h>
#include <phx_uart_bridge/MotorMessage.h>
#include <phx_workstation_override/WorkstationOverrideConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

/**
 * Controls the motor manually via dynamic reconfigure.
 */
class ManualMotorControl : public WorkstationOverrideInstruction {

public:

	ManualMotorControl(WOIPublisher & woiPublisher);
	void updateConfig(phx_workstation_override::WorkstationOverrideConfig &config, uint32_t level);
	void runOverride();
	void setControl(int motor1,int motor2,int motor3,int motor4);

private:

	phx_uart_bridge::MotorMessage msg;
	int lowVal;
	int highVal;
	bool overrideEnabled;
    	dynamic_reconfigure::Server<phx_workstation_override::WorkstationOverrideConfig> reconfigureServer;
    	dynamic_reconfigure::Server<phx_workstation_override::WorkstationOverrideConfig>::CallbackType reconfigureCallback;



};


#endif /* MANUALMOTORCONTROL_H_ */
