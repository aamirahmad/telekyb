/*
 * JoystickOptions.cpp
 *
 *  Created on: Oct 24, 2011
 *      Author: mriedel
 */

#include <telekyb_hid/JoystickOptions.hpp>

namespace TELEKYB_NAMESPACE
{

//template<> JoystickOptions* Singleton<JoystickOptions>::instance = NULL;

JoystickOptions::JoystickOptions()
	: OptionContainer("Joystick")
{
	tJoystickConfigFile = addOption<std::string>("tJoystickConfigFile",
			"YAML Config File for Joysticks", std::string(TELEKYB_HID_PROJECT_SOURCE_DIR) + "/config/joysticks.yaml", false, true);
	tJoystickUseProductIDForRosPath = addOption<bool>("tJoystickUseProductIDForRosPath",
			"Use ProductID reported by USB as Publishing Name", false, false, true);
	tDeadZone = addBoundsOption<double>("tDeadZone",
			"Amount by which the joystick has to move before it is considered to be off-center",
			0.05, 0.0, 1.0, false, false);
	tPubName = addOption<std::string>("tPubName",
			"TopicName that get's added to NodeHandle",
			"joy", false, true);
	tButtonRemapping = addOption<std::vector<int> >("tButtonRemapping",
			"Button Remapping e.g. [1,0,3,3] switches 1 and 0 and maps 3 to 3 AND 4",
			std::vector<int>(), false, true);
	tAxesRemapping = addOption<std::vector<int> >("tAxesRemapping",
			"Axes Remapping e.g. [1,0,3,3] switches 1 and 0 and maps 3 to 3 AND 4",
			std::vector<int>(), false, true);
	tAxisMultiplier = addOption<std::vector<double> >("tAxisMultiplier",
			"Axes Value Multiplication e.g. [-1.0,0.0,0.5] inverts 0, set 1 to 0 scales 2",
			std::vector<double>(), false, true);
}

}
