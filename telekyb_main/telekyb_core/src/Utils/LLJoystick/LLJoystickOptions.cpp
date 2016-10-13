/*
 * LLJoystickOptions.cpp
 *
 *  Created on: Oct 29, 2011
 *      Author: mriedel
 */

#include "LLJoystickOptions.hpp"

namespace TELEKYB_NAMESPACE {

LLJoystickOptions::LLJoystickOptions()
	: OptionContainer("LLJoystick")
{
	tJoystickTopic = addOption<std::string>("tJoystickTopic","Joystick Topic with sensor_msgs::Joy",
			"/TeleKyb/tJoy/joy", false, true);
	tLLCommandsTopic = addOption<std::string>("tLLCommandsTopic","Topic for publishing telekyb_msgs::LLCommands",
			"/TeleKyb/SwarmSimX/0/commands", false, true);

	tCommandRate = addOption<int>("tCommandRate","Rate at which LLCommands are send in (Hz)",
			100, false, true);

}


}
