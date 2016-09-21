/*
 * MKInterfaceConnectionOptions.cpp
 *
 *  Created on: Nov 27, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface_outdoor/MKInterfaceConnectionOptions.hpp>

namespace TELEKYB_NAMESPACE {

MKInterfaceConnectionOptions::MKInterfaceConnectionOptions()
	: OptionContainer("MKInterfaceConnection")
{
	tBaudRate = addOption< BaudRateBaseEnum<size_t>::Type >("tBaudRate",
			"Defines the BaudRate of the Serial Connetion", BaudRate::BAUD115200, false, true);
	tTermiosCFlags = addOption< int >("tTermiosCFlags",
			"Defines the Termios CFlags. Beware, this is hard to set from the command line!",
			CS8 | CLOCAL | CREAD , false, true);

	tAsyncSendFrequency = addBoundsOption<double>("tAsyncSendFrequency", "Max Frequency to Send Set/Update Requests",
			100.0,1.0,100.0, false, false);


}

} /* namespace telekyb */
