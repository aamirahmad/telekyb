/*
 * HandJoystickOptions.cpp
 *
 *  Created on: Jan 5, 2012
 *      Author: mriedel
 */

#include "HandJoystickOptions.hpp"

namespace TELEKYB_NAMESPACE {

HandJoystickOptions::HandJoystickOptions()
	: OptionContainer("HandJoystick")
{
	tJoystickTopic = addOption<std::string>("tJoystickTopic", "Joysticktopic to use", "/TeleKyb/tJoy/joy", false, true);
	tTransformStampedTopic = addOption<std::string>("tTransformStampedTopic", "Transform to use", "undef", true, true);
	tJoyPubName = addOption<std::string>("tJoyPubName", "Pub of Joystick Msg", "joy", false, true);
	tTransformPubName = addOption<std::string>("tTransformPubName", "Pub of Transform Msg", "HandJoystickTransform", false, true);
	tChildFrameID = addOption<std::string>("tChildFrameID", "Child Frame ID for Transforms",
			"world", false, true);
	tMaxAxisValue = addOption<double>("tMaxAxisValue", "Max Axis Value", 1.0, false, true);
	tAxisValueScale = addOption<double>("tAxisValueScale", "Scales the Input by that factor", 5.0, false, true);
}


} /* namespace telekyb */
