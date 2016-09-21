/*
 * TrajectoryProcessorControllerOptions.hpp
 *
 *  Created on: Dec 13, 2011
 *      Author: mriedel
 */

#ifndef TRAJECTORYPROCESSORCONTROLLEROPTIONS_HPP_
#define TRAJECTORYPROCESSORCONTROLLEROPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class TrajectoryProcessorControllerOptions : public OptionContainer {
public:
	Option<double>* tInitialStateTimeout;
	Option< std::vector<std::string> >* tTrajectoryModules;
	Option<std::string>* tStateEstimationTopic;
	TrajectoryProcessorControllerOptions();
};

} /* namespace telekyb */
#endif /* TRAJECTORYPROCESSORCONTROLLEROPTIONS_HPP_ */
