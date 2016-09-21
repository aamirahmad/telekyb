/*
 * VRPNTrackerServerOptions.hpp
 *
 *  Created on: Jan 9, 2012
 *      Author: mriedel
 */

#ifndef VRPNTRACKERSERVEROPTIONS_HPP_
#define VRPNTRACKERSERVEROPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>
#include <telekyb_base/Base/Singleton.hpp>

#include <telekyb_base/Spaces/R3.hpp>

namespace TELEKYB_NAMESPACE {

class VRPNTrackerServerOptions : public OptionContainer, public Singleton<VRPNTrackerServerOptions> {
public:
	Option< std::vector<std::string> >* tVRPNTopicNames;
	Option<Eigen::Matrix3d>* tViconToNEDMatrix;
	VRPNTrackerServerOptions();
};

}

#endif /* VRPNTRACKERSERVEROPTIONS_HPP_ */
