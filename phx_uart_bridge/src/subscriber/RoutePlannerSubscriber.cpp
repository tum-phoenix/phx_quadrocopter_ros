/*
 * RoutePlannerSubscriber.cpp
 *
 *  Created on: 28.10.2013
 *      Author: ic3
 */

#include <phx_uart_bridge/subscriber/RoutePlannerSubscriber.h>

RoutePlannerSubscriber::RoutePlannerSubscriber(ros::NodeHandle & nh) {

	std::string topic;
	nh.param("routePlannerTopic", topic, std::string("/cmd_vel"));
	subscriber = nh.subscribe(topic, 1, &RoutePlannerSubscriber::processMessageCb, this);

	ROS_INFO("Initialized RoutePlannerSubscriber");
}

void RoutePlannerSubscriber::processMessageCb(const geometry_msgs::Twist& msg) {
	message = msg;
}

bool RoutePlannerSubscriber::overrideSensor(phx_uart_bridge::SensorMessage & sensorMessage) const {

	/**
	 * Twist.linear.x: AccX
	 * Twist.linear.y: AccY
	 * Twist.linear.z: AccZ
	 */

	sensorMessage.radio1 = message.linear.x;
	sensorMessage.radio2 = message.linear.y;
	sensorMessage.radio3 = message.linear.z;
	return true;
}
