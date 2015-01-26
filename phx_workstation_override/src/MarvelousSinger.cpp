/*
 * MarvelousSinger.cpp
 *
 *  Created on: 01.11.2013
 *      Author: ic3
 */

#include <phx_workstation_override/MarvelousSinger.h>
#include <ctype.h>

std::string MarvelousSinger::AlleMeineEntchen = "3C2 1D2 1E2 1F2 1G2 1G2 1A2 1A2 1A2 1A2 1G2 1A2 1A2 1A2 1A2 1G2 1F2 1F2 1F2 1F2 1E2 1E2 1D2 1D2 1D2 1D2 1C";
std::string MarvelousSinger::ImperialMarch = "3G 3G 3G 3Eb 3Bb 3G 3Eb 3Bb 3G 3D 3D 3D 3Eb 3Bb 3G 3Eb 3Bb 3G 3G 3G 3G 3G 3F# 3F 3E 3Eb 3E 3Ab 3C# 3C 3B 3Bb 3A 3Bb 3Eb 3F# 3Bb 3G 3Bb 3D 3G 3G 3G 3G 3F# 3F 3E 3Eb 3E 3Ab 3C# 3C 3B 3Bb 3A 3Bb 3Eb 3Bb 3GEb 3Bb 3G";

MarvelousSinger::MarvelousSinger(WOIPublisher & woiPublisher) :
	WorkstationOverrideInstruction(woiPublisher) {

}

void MarvelousSinger::runOverride() {

	// Determine the song that the user wants to be sung
	std::cout << "1: Alle meine Entchen" << std::endl;
	std::cout << "2: Imperial March" << std::endl;
	std::cout << ">> ";
	std::string chosenSongInput;
	std::getline(std::cin, chosenSongInput);
	int chosenSong = atoi(chosenSongInput.c_str());
	switch (chosenSong) {
		case 1:
			setSong(MarvelousSinger::AlleMeineEntchen);
			break;

		case 2:
			setSong(MarvelousSinger::ImperialMarch);
			break;

		default:
			std::cout << "Unknown id";
			return;
	}

	// Now that we know the song, start overriding hte motors
	setOverrideFlags(true, false);

	// Sing the song as long as it takes or ros::ok() is false
	double frequency = 0;
	ros::Duration length;
	phx_uart_bridge::MotorMessage msg;
	for( std::string::iterator itr = song.begin(); itr < song.end() && ros::ok(); itr++ ) {

		// Parse the length of this note
		length = getLength(itr);

		// Parse the frequency of this note
		frequency = getFrequency(itr);

		// Now play the tune
		ROS_INFO("Next tune: Length is %d.%.9d, frequency is %f", length.sec, length.nsec, frequency);
		updateMotorData(msg, frequency);
		motorPub.publish(msg);

		length.sleep();
	}

	ROS_INFO("Song is sung");
	setOverrideFlags(false, false);
}

ros::Duration MarvelousSinger::getLength(std::string::iterator & itr) {

	int denominator = *itr - 48;
	itr++;

//	ROS_INFO("Denominator is %d", denominator);

	// Sanity check
	if ( denominator < 10 )
		return ros::Duration( denominator * .25 );

	ROS_WARN("Dominator is %d, defaulting to .75 seconds duration.", denominator);
	return ros::Duration(.75);

}

double MarvelousSinger::getFrequency(std::string::iterator & itr) {

	// First letter is always the tune, e.g. A, H, C, ...
	// UTF-8-Value - 65	: actual integer value
	// value * 2		: One note consists of two half-notes
	// + 1				: We start with A = 1
	double val = (toupper(*itr) - 65) * 2 + 1;
//	ROS_INFO("");
//	ROS_INFO("Base tune: %d -> %f", *itr, val);

	// Next letter might be a modifier: # or b
	itr++;
	if ( *itr == '#' || *itr == 'b' ) {
		val += ( *itr == '#' ) ? 1 : -1;
//		ROS_INFO("Half-tune modifier: %2d (%d) -> %f", ( *itr == '#' ) ? 1 : -1, *itr, val);
		itr++;
	}
	else
		ROS_INFO("Half-tune modifier: None");

	// Next letter might be an octave modifier (digit)
	if ( isdigit(*itr) ) {
		val += 12 * (*itr - 48);
//		ROS_INFO("Octave modifier: %d (%d) -> %f", *itr, 12 * (*itr - 48), val);
		itr++;
	}

	// Now convert the note-representation to a frequency
//	val = 440 * pow(2, (val - 49) / 12.0 );
//	ROS_INFO("Frequency: %f", val);

	return val;
}

void MarvelousSinger::updateMotorData(phx_uart_bridge::MotorMessage & msg, double & frequency) {
	// Motor goes from 2200 to 3600					-> 1400

	// Step go from 1 to 88							-> 88
	// Motor stepsize								-> 15,9
	int val = 2200 + ( frequency *  15.9 );

	// Frequency goes from 27,5000 to 4186,01		-> 4158,51
	// Motor stepsize								->
//	int val =

	ROS_INFO("Val is: %d", val);

	msg.motor0 = val;
	msg.motor1 = val;
	msg.motor2 = val;
	msg.motor3 = val;
}

void MarvelousSinger::setSong(std::string song) {
	this->song = song;
}
