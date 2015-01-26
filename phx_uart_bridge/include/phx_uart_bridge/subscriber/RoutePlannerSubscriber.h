/*
 * RoutePlannerSubscriber.h
 *
 *  Created on: 28.10.2013
 *      Author: ic3
 */

#ifndef ROUTEPLANNERSUBSCRIBER_H_
#define ROUTEPLANNERSUBSCRIBER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <phx_uart_bridge/SensorMessage.h>
#include <phx_uart_bridge/MessageOverrider.h>

class RoutePlannerSubscriber : public MessageOverrider {
public:

	RoutePlannerSubscriber(ros::NodeHandle & nh);
	virtual ~RoutePlannerSubscriber() { }

	bool overrideSensor(phx_uart_bridge::SensorMessage & sensorMessage) const;

private:

	ros::Subscriber subscriber;
	geometry_msgs::Twist message;

	void processMessageCb(const geometry_msgs::Twist& msg);

};

#endif /* ROUTEPLANNERSUBSCRIBER_H_ */
