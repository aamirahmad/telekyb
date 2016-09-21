/*
 * VRPNTrackerClientOptions.hpp
 *
 *  Created on: Dec 11, 2011
 *      Author: mriedel
 */

#ifndef VRPNTRACKERCLIENTOPTIONS_HPP_
#define VRPNTRACKERCLIENTOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>
#include <telekyb_base/Base/Singleton.hpp>

namespace TELEKYB_NAMESPACE {

class VRPNTrackerClientOptions : public OptionContainer, public Singleton<VRPNTrackerClientOptions> {
public:
	Option<std::string>* tVRPNHostname;
	Option<std::string>* tChildFrameID;
	Option<bool>* tEnableTF;
	Option< std::vector<std::string> >* tVRPNClientObjects;
	Option<Eigen::Matrix3d>* tVRPNRotationMatrix;
	VRPNTrackerClientOptions();
};

} /* namespace telekyb */
#endif /* VRPNTRACKERCLIENTOPTIONS_HPP_ */
