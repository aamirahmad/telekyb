/*
 * joy.cpp
 *
 *  Created on: Oct 25, 2011
 *      Author: mriedel
 */


#include <telekyb_base/TeleKyb.hpp>

#include <telekyb_joystick/Joystick.hpp>
#include <telekyb_joystick/JoystickOptions.hpp>

using namespace telekyb;

int main(int argc, char* argv[])
{
	// Joystick is only output. Does not need to spin. // TODO: is this true?
	RawOptionsContainer::addOption("tRosNrSpinnerThreads","-1");
	TeleKyb::init(argc,argv,"tJoy", ros::init_options::AnonymousName);
	// create Option Singleton
	JoystickOptions* options = JoystickOptions::InstancePtr();

	Joystick j(options->tDevicePath->getValue(), true);
	j.run();

	j.closeJoystick();

	//delete options;

	delete options;

	// it is important to free everything before this call.

	TeleKyb::shutdown();
	return 0;
}

