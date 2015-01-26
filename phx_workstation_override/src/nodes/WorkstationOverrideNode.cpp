/*
 * WorkstationOverrideNode.cpp
 *
 *  Created on: 01.11.2013
 *      Author: ic3
 */

#include <phx_workstation_override/WorkstationOverrideNode.h>
#include <phx_workstation_override/WorkstationOverrideParams.h>
#include <std_msgs/String.h>

#define	WO_EMERGERNY_STOP			"0"
#define WO_INITIAL_MOTOR_TEST		"1"
#define WO_MANUAL_MOTOR_CONTROL		"2"
#define WO_MARVELOUS_SINGER			"3"
#define WO_USER_MOTOR_OVRRIDE		"4"
#define WO_EXIT						"5"
#define WO_COLOR_DEFAULT			"\e[0m"

int main(int argc, char **argv) {
	ROS_INFO("Starting WorkstationOverrideNode");

	// Initialize ROS
	ros::init(argc, argv, "WorkstationOverrideNode");
	ros::NodeHandle nh("WorkstationOverrideNode");

	// Initialize the publisher
	WOIPublisher woiPublisher;
	woiPublisher.activatePublisher = nh.advertise<std_msgs::String>(WorkstationOverrideParams::TOPIC_ACTIVATE, 1);
	woiPublisher.motorPublisher = nh.advertise<phx_uart_bridge::MotorMessage>(WorkstationOverrideParams::TOPIC_MOTOR, 1);
	woiPublisher.sensorPublisher = nh.advertise<phx_uart_bridge::SensorMessage>(WorkstationOverrideParams::TOPIC_SENSOR, 1);

	// Initialize the override instructions
	EmergencyStopOverride emergencyStopOverride(woiPublisher);
	InitialMotorTest initialMotorTest(woiPublisher);
	ManualMotorControl manualMotorControl(woiPublisher);
	MarvelousSinger marvelousSinger(woiPublisher);
	UserMotorOverride userMotorOverride(woiPublisher);

//	marvelousSinger.setSong("1a4 2a#4 3bb4 4b4");
//	marvelousSinger.setSong(MarvelousSinger::AlleMeineEntchen);

	while (ros::ok()) {

		int choice = getUserChoice();
		std::cout << std::endl << "---------------------------------------------------------------" << std::endl;

		if ( choice == availableOverrides.exit.id )
			ros::shutdown();

		if ( choice == availableOverrides.emergencyOverride.id )
			emergencyStopOverride.runOverride();

		else if ( choice == availableOverrides.initialMotorTest.id )
			initialMotorTest.runOverride();

		else if ( choice == availableOverrides.manualMotorOverride.id )
			manualMotorControl.runOverride();

		else if ( choice == availableOverrides.marvelousSinger.id )
			marvelousSinger.runOverride();

		else if ( choice == availableOverrides.userMotorOverride.id )
			userMotorOverride.runOverride();

		ros::spinOnce();
	}

    ROS_INFO("Quitting WorkstationOverrideNode");
	return 0;
}

int getUserChoice() {

	std::cout << std::endl << "--- WorkstationOverride: Menu ---------------------------------" << std::endl;
	std::cout << std::string(6, '\n');

	int choice = 0;
	while ( ros::ok() ) {
		printChoices(choice);

		int input = linux_getch();
		if ( input == KB_UP )
			choice -= ( choice > 0 ) ? 1 : 0;

		else if ( input == KB_DOWN )
			choice += ( choice < availableOverrides.exit.id ) ? 1 : 0;

		else if ( input == KB_ESC )
			return availableOverrides.exit.id;

		else if ( input == KB_ENTER )
			return choice;
	}
	return availableOverrides.exit.id;
}

void printChoices(int & activeChoice) {

	// Clear the previous output
	std::cout << '\r';
	for ( int i = 0; i < 6; i++ )
		std::cout << "\033[F" << "\033[J";

	int currentChoice = 0;

	printChoice(availableOverrides.emergencyOverride, (currentChoice++ == activeChoice));
	printChoice(availableOverrides.initialMotorTest, (currentChoice++ == activeChoice));
	printChoice(availableOverrides.manualMotorOverride, (currentChoice++ == activeChoice));
	printChoice(availableOverrides.marvelousSinger, (currentChoice++ == activeChoice));
	printChoice(availableOverrides.userMotorOverride, (currentChoice++ == activeChoice));
	printChoice(availableOverrides.exit, (currentChoice++ == activeChoice));
}

void printChoice(AvailableOverrides::AvailableOverride & o, bool active) {
	if ( active )
		std::cout << colors.Selected << " > ";
	else
		std::cout << "   ";
	std::cout << std::left << std::setw(50) << o.label << colors.Default << std::endl;
}

int linux_getch(void) {
	struct termios oldstuff;
	struct termios newstuff;
	int inch;

	tcgetattr(STDIN_FILENO, &oldstuff);
	newstuff = oldstuff;							// save old attributes
	newstuff.c_lflag &= ~(ICANON | ECHO); 			// reset "canonical" and "echo" flags*/
	tcsetattr(STDIN_FILENO, TCSANOW, &newstuff);	// set new attributes
	inch = getchar();
	if ( inch == 91 || inch == 27 )					// Escape character, hence special key
		inch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldstuff);	// restore old attributes

	if (inch == END_FILE_CHARACTER)
		inch = EOF;
	return inch;
}

