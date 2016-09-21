/*
 * MKROSInterfaceOptions.cpp
 *
 *  Created on: Nov 29, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface_outdoor/MKROSInterfaceOptions.hpp>

namespace TELEKYB_NAMESPACE {

MKROSInterfaceOptions::MKROSInterfaceOptions()
	: OptionContainer("MKROSInterface")
{
// 	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	RCCommandsTopic = addOption<std::string>("RCCommandsTopic","Topic for subscibing to telekyb_msgs::TKCommands",
			"undef", false, true);
	operatorRequestTopic = addOption<std::string>("operatorRequestTopic",
			"Topic for subscribing to the operator Interruption", "undef", false, true);
// 	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@	
	
	tCommandsTopic = addOption<std::string>("tCommandsTopic","Topic for subscibing to telekyb_msgs::TKCommands",
			"undef", true, true);
	tBatteryUpdatePeriod = addBoundsOption("tBatteryUpdatePeriod",
			"Update the Battery value every X seconds", 5.0, 0.1, 60.0, false, true);
	tAtmoPressUpdatePeriod = addBoundsOption("tAtmoPressUpdatePeriod",
			"Update the AtmoPress value every X seconds", 0.5, 0.1, 60.0, false, true);
	tYawRateDrift = addOption("tYawRateDrift",
			"constant drift compensation for the yaw rate", 0.0, false, false); // Get this value by computing the mean yaw rate command given during hovering in position mode
	tEmergencyLandService = addOption<std::string>("tEmergencyLandService",
			"What std_srvs::Empty to call when Battery is empty.", "undef", true, true);
}


} /* namespace telekyb */
