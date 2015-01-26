/*
 * MarvelousSinger.h
 *
 *  Created on: 01.11.2013
 *      Author: ic3
 */

#ifndef MARVELOUSSINGER_H_
#define MARVELOUSSINGER_H_

#include <phx_workstation_override/WorkstationOverrideInstruction.h>
#include <phx_uart_bridge/MotorMessage.h>

#include <ros/ros.h>

/**
 * Performs a given song using the motors.
 */
class MarvelousSinger: public WorkstationOverrideInstruction {

public:

	static std::string AlleMeineEntchen;
	static std::string ImperialMarch;

	MarvelousSinger(WOIPublisher & woiPublisher);

	/**
	 * Sets the song which will be played the next time.
	 *
	 * The string has to be well formatted, e.g. "A0 C4" or "D#1 Fb2"
	 */
	void setSong(std::string song);

	void runOverride();

private:

	std::string song;

	/**
	 * Calculates the length of the tune. Iterator will point to the next character afterwards.
	 */
	ros::Duration getLength(std::string::iterator & itr);

	/**
	 * Calculates the frequency of the tune, starting at the current iterator-position.
	 * Iterator will point to the next character afterwards (probably a blank).
	 */
	double getFrequency(std::string::iterator & itr);

	/**
	 * Updates the given MotorMessage for the specified frequency
	 */
	void updateMotorData(phx_uart_bridge::MotorMessage & msg, double & frequency);
};

#endif /* MARVELOUSSINGER_H_ */
