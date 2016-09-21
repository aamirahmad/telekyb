/*
 * joy.cpp
 *
 *  Created on: Oct 24, 2011
 *      Author: mriedel
 */


#include <telekyb_hid/Joystick.hpp>
#include <telekyb_base/TeleKyb.hpp>

#include <telekyb_hid/JoystickOptions.hpp>

#include <hidapi.h>

using namespace telekyb;

int main(int argc, char* argv[])
{
	TeleKyb::init(argc, argv, "tJoy");

	// Singleton Creation
	JoystickOptions* jO = new JoystickOptions();


	std::vector<Joystick*> joysticks;
	Joystick::getJoysticks(jO->tJoystickConfigFile->getValue(), joysticks);

	for (unsigned int i = 0; i < joysticks.size(); ++i) {
		ROS_INFO_STREAM("Starting Joystick Thread for " << joysticks[i]->getName());
		joysticks[i]->startThread();
	}

	//ros::waitForShutdown();


	// cleanup
	for (unsigned int i = 0; i < joysticks.size(); ++i) {
		joysticks[i]->joinThread();
		delete joysticks[i];
	}

	delete jO;

	TeleKyb::shutdown();
	return 0;
}

