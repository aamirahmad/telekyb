/*
 * JoystickOptions.cpp
 *
 *  Created on: Oct 25, 2011
 *      Author: mriedel
 */

#include <telekyb_joystick/JoystickOptions.hpp>

namespace TELEKYB_NAMESPACE
{

//template<> JoystickOptions* Singleton<JoystickOptions>::instance = NULL;

JoystickOptions::JoystickOptions()
	: OptionContainer("JoystickOptions")
{
	tDevicePath = addOption<std::string>("tDevicePath",
			"Device Path of Joystick",
			"/dev/input/js0", false, true);
	tDeadZone = addBoundsOption<double>("tDeadZone",
			"Amount by which the joystick has to move before it is considered to be off-center",
			0.05, -1.0, 1.0, false, false);
	tAutoRepeatRate = addOption<double>("tAutoRepeatRate",
			"Rate in Hz at which a joystick that has a non-changing state will resend the previously sent message",
			0.0 , false, false);
	tCoalesceInterval = addOption<double>("tCoalesceInterval",
			"Axis events that are received within coalesce_interval (seconds) of each other are sent out in a single ROS message",
			0.001 , false, false);
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
	tPublishVector3 = addOption< bool >("tPublishVector3",
			"Publish geometry_msgs::Vector3Stamped of first 3 axes [x,y,z] to tPubName_vector3",
			false, false, true);
}

}
