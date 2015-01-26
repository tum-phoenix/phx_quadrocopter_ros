/*
 * SensorMessageMerger.cpp
 *
 *  Created on: 15.07.2014
 *      Author: ic3
 */

#include <phx_uart_bridge/subscriber/KinectSubscriber.h>
#include <phx_uart_bridge/subscriber/RoutePlannerSubscriber.h>
#include <phx_uart_bridge/subscriber/WorkstationOverrideSubscriber.h>
#include <phx_uart_bridge/SensorMessageMerger.h>
#include <phx_uart_bridge/UARTBridgeParams.h>

SensorMessageMerger::SensorMessageMerger(ros::NodeHandle & nh) {

	publisher = nh.advertise<phx_uart_bridge::SensorMessage>(UARTBridgeParams::TOPIC_SENSOR, 1);

	// Create the message override instances, highest priority first.
	WorkstationOverrideSubscriber s3(nh);
	overriders.push_back(s3);

	RoutePlannerSubscriber s1(nh);
	overriders.push_back(s1);

	KinectSubscriber s0(nh);
	overriders.push_back(s0);

	// Register the callback
	nh.subscribe(UARTBridgeParams::TOPIC_SENSOR_RAW, 1, &SensorMessageMerger::rawSensorMessageCb, this);
}

void SensorMessageMerger::rawSensorMessageCb(const phx_uart_bridge::SensorMessagePtr & raw) {

	// Update the local object
	msg.radio0 = raw->radio0;
	msg.radio1 = raw->radio1;
	msg.radio2 = raw->radio2;
	msg.radio3 = raw->radio3;
	msg.accX = raw->accX;
	msg.accY = raw->accY;
	msg.accZ = raw->accZ;
	msg.angleSpeedX = raw->angleSpeedX;
	msg.angleSpeedY = raw->angleSpeedY;
	msg.angleSpeedZ = raw->angleSpeedZ;
	msg.battery = raw->battery;

	// Only include external data if radio controller is not active
	if ( msg.radio0 == 0 )

		// Iterate through all overridering instances. Abort once it has been overridden.
		for (std::vector<MessageOverrider>::const_iterator itr = overriders.begin(); itr != overriders.end(); ++itr )
			if (itr->overrideSensor(msg))
				break;

	// Eventually publish the (updated) message
	publisher.publish(msg);
}
