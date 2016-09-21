/*
 * SerialCommandDeviceOptions.cpp
 *
 *  Created on: Jul 12, 2012
 *      Author: mriedel
 */

#include "SerialCommandDeviceOptions.hpp"


namespace TELEKYB_NAMESPACE {

SerialCommandDeviceOptions::SerialCommandDeviceOptions()
	: OptionContainer("SerialCommandDevice")
{
	tDeviceName = addOption< std::string >("tDeviceName",
			"Device Name to connect to!",
			"undef" , true, true);

	// Topic Name
	tTopicName = addOption< std::string >("tTopicName",
			"Topic to receive Command Messages",
			"MotorCommands" , false, true);

	tBaudRate = addOption< BaudRateBaseEnum<size_t>::Type >("tBaudRate",
			"Defines the BaudRate of the Serial Connetion", BaudRate::BAUD230400, false, true);
	tTermiosCFlags = addOption< int >("tTermiosCFlags",
			"Defines the Termios CFlags. Beware, this is hard to set from the command line!",
			CS8 | CLOCAL | CREAD , false, true);

	minTimeStep = addOption<double>("minTimeStep", "Minimum time step between two commands", 1.0/600.0, false, true);
	motorNumber = addOption<int>("servoNumber", "Number of motors in the bus", 4, false, true);
}

}
