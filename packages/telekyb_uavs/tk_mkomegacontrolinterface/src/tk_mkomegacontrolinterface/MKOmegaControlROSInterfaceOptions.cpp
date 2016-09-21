/*
 * MKOmegaControlROSInterfaceOptions.cpp
 *
 *  Created on: Nov 29, 2011
 *      Author: mriedel
 */

#include <tk_mkomegacontrolinterface/MKOmegaControlROSInterfaceOptions.hpp>

namespace TELEKYB_NAMESPACE {

MKOmegaControlROSInterfaceOptions::MKOmegaControlROSInterfaceOptions()
	: OptionContainer("MKOmegaControlROSInterface")
{
	tCommandsTopic = addOption<std::string>("tCommandsTopic","Topic for subscibing to telekyb_msgs::TKCommands",
			"undef", true, true);
	tBatteryUpdatePeriod = addBoundsOption("tBatteryUpdatePeriod",
			"Update the Battery value every X seconds", 5.0, 0.1, 60.0, false, true);
	tEmergencyLandService = addOption<std::string>("tEmergencyLandService",
			"What std_srvs::Empty to call when Battery is empty.", "undef", true, true);
}


} /* namespace telekyb */
