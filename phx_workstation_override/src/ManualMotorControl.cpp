#include <phx_workstation_override/ManualMotorControl.h>
#include <iostream>

ManualMotorControl::ManualMotorControl(WOIPublisher & woiPublisher) :
		WorkstationOverrideInstruction(woiPublisher), reconfigureCallback(
				boost::bind(&ManualMotorControl::updateConfig, this, _1, _2)) {
	reconfigureServer.setCallback(reconfigureCallback);
	lowVal = 0;
	highVal = 1;
}

void ManualMotorControl::updateConfig(phx_workstation_override::WorkstationOverrideConfig &config, uint32_t level) {
    if (overrideEnabled) {
		setControl(config.motor0, config.motor1, config.motor2, config.motor3);
	}
}

void ManualMotorControl::runOverride() {
	overrideEnabled = true;
}

void ManualMotorControl::setControl(int motor1, int motor2, int motor3, int motor4) {

	setOverrideFlags(true, false);
	msg.motor0 = motor1;
	msg.motor1 = motor2;
	msg.motor2 = motor3;
	msg.motor3 = motor4;
	motorPub.publish(msg);
	setOverrideFlags(false, false);
}
