/*
 * WorkstationOverrideInstruction.h
 *
 *  Created on: 01.11.2013
 *      Author: ic3
 */

#ifndef WORKSTATIONOVERRIDEINSTRUCTION_H_
#define WORKSTATIONOVERRIDEINSTRUCTION_H_

#define TPWO_MOTOR	0x1
#define TPWO_SENSOR 0x2

#include <ros/ros.h>
#include <std_msgs/String.h>

struct WOIPublisher {
	ros::Publisher activatePublisher;
	ros::Publisher motorPublisher;
	ros::Publisher sensorPublisher;
};

class WorkstationOverrideInstruction {

public:
	explicit WorkstationOverrideInstruction(ros::Publisher & activatePub, ros::Publisher & motorPub, ros::Publisher & sensorPub);
	explicit WorkstationOverrideInstruction(WOIPublisher & woiPublisher);
	virtual ~WorkstationOverrideInstruction();

protected:

	ros::Publisher activatePub;
	ros::Publisher motorPub;
	ros::Publisher sensorPub;

	/**
	 * Publishes the given override flags.
	 */
	void setOverrideFlags(bool motor, bool sensor);

	virtual void runOverride() = 0;

};


#endif /* WORKSTATIONOVERRIDEINSTRUCTION_H_ */
