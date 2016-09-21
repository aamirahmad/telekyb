/*
 * SerialIMUDeviceOptions.cpp
 *
 *  Created on: Jul 12, 2012
 *      Author: mriedel
 */

#include "SerialIMUDeviceOptions.hpp"


namespace TELEKYB_NAMESPACE {

SerialIMUDeviceOptions::SerialIMUDeviceOptions()
	: OptionContainer("SerialIMUDevice")
{
	tDeviceName = addOption< std::string >("tDeviceName",
			"Device Name to connect to!",
			"undef" , true, true);

//	tTerminalChars = addOption< std::string >("tTerminalChars",
//			"Terminal Characters for ThreadedSerialDevice",
//			"\n\r" , false, true);

	tBaudRate = addOption< BaudRateBaseEnum<size_t>::Type >("tBaudRate",
			"Defines the BaudRate of the Serial Connetion", BaudRate::BAUD230400, false, true);
	tTermiosCFlags = addOption< int >("tTermiosCFlags",
			"Defines the Termios CFlags. Beware, this is hard to set from the command line!",
			CS8 | CLOCAL | CREAD , false, true);
}

}
