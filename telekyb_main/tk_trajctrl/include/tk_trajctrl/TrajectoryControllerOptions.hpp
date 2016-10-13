/*
 * TrajectoryControllerOptions.hpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#ifndef TRAJECTORYCONTROLLEROPTIONS_HPP_
#define TRAJECTORYCONTROLLEROPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class TrajectoryControllerOptions : public OptionContainer {
public:
	Option<std::string>* tPluginLookupName;

	Option<bool>* tDoMassEstimation;
	Option<bool>* tDoInertiaMatrixEstimation;

	Option<std::string>* tStateEstimationTopic;


	TrajectoryControllerOptions();
};

}

#endif /* TRAJECTORYCONTROLLEROPTIONS_HPP_ */
