/*
 * DHDDeviceOptions.cpp
 *
 *  Created on: Mar 4, 2012
 *      Author: mriedel
 */

#include "DHDDeviceOptions.hpp"

namespace TELEKYB_NAMESPACE {

DHDDeviceOptions::DHDDeviceOptions(const std::string& identifier)
	: OptionContainer("DHDDevice_" + identifier)
{
	tHapticDeviceController = addOption<std::string>("tHapticDeviceController",
			"Specifiy the Controller to use for this Device", "tk_haptics_base/SampleController", true, true);
	tEnableForceAtStart = addOption<bool>("tEnableForceAtStart",
			"Automatically enable Force at Startup", true, false, true);
	tCustomEffectorMass = addOption<double>("tCustomEffectorMass",
			"Use this value as the EffectorMass. Negative Values: Use Devices Default Setting", -1.0, false, true);
	tDisableGravityCompensation = addOption<bool>("tDisableGravityCompensation",
			"Set to true to disable Gravity Compenstation", false, false, true);
	tCenterTranslation = addOption<Position3D>("tCenterTranslation",
			"Translate the Haptic Device Center (in m)", Position3D(0.0, 0.0, 0.0), false, true);
	tForceOffset = addOption<Vector3D>("tForceOffset",
			"Add constant force term to the applied force value", Vector3D(0.0, 0.0, 0.0), false, true);

	// Informational output
	tStatusOutputFreq = addOption<double>("tStatusOutputFreq",
			"Frequency at which to output tk_haptics_msgs::TKHapticOutput. < 0.01 disables", 0.0, false, false);
	tStatusOutputTopic = addOption<std::string>("tStatusOutputTopic",
			"Topicname of tk_haptics_msgs::TKHapticOutput Msg", "StatusOutput", false, true);
}


}
