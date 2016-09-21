/*
 * VRPNTrackerClientOptions.cpp
 *
 *  Created on: Dec 11, 2011
 *      Author: mriedel
 */

#include <telekyb_vrpn/VRPNTrackerClientOptions.hpp>

namespace TELEKYB_NAMESPACE {

//template<> VRPNTrackerClientOptions* Singleton<VRPNTrackerClientOptions>::instance = NULL;

VRPNTrackerClientOptions::VRPNTrackerClientOptions()
	: OptionContainer("VRPNTrackerClient")
{
	tVRPNHostname = addOption<std::string>("tVRPNHostname", "The host address of the VPRN Tracker Server.",
			"192.168.0.100", false, true);
	tChildFrameID = addOption<std::string>("tChildFrameID", "Child Frame ID for Transforms",
			"world", false, true);
	tEnableTF = addOption<bool>("tEnableTF", "Send tf Transform Data.",
			true, false, true);
	tVRPNClientObjects = addOption< std::vector<std::string> >("tVRPNClientObjects",
			"Objects to track.", std::vector<std::string>(), true, true);
	tVRPNRotationMatrix = addOption<Eigen::Matrix3d>("tVRPNRotationMatrix",
			"Rotation matrix to bring base frame to East-North-Up convention", Eigen::Matrix3d::Identity(), false, true);

}

}
