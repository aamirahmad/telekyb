/*
 * VRPNTrackerServerOptions.cpp
 *
 *  Created on: Jan 9, 2012
 *      Author: mriedel
 */

#include <telekyb_vrpn/VRPNTrackerServerOptions.hpp>

namespace TELEKYB_NAMESPACE {

//template<> VRPNTrackerServerOptions* Singleton<VRPNTrackerServerOptions>::instance = NULL;

VRPNTrackerServerOptions::VRPNTrackerServerOptions()
	: OptionContainer("VRPNTrackerServer")
{
	tVRPNTopicNames = addOption< std::vector<std::string> >("tVRPNTopicNames",
			"Topics to publish", std::vector<std::string>(), true, true);

	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	m.diagonal() << 1.0,-1.0,-1.0;
	tViconToNEDMatrix = addOption<Eigen::Matrix3d>("tViconToNEDMatrix","ConversionMatrix from Vicon to NED", m,false,true);

}

}
