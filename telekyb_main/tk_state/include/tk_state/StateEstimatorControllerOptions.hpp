/*
 * StateEstimatorControllerOptions.hpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#ifndef STATEESTIMATORCONTROLLEROPTIONS_HPP_
#define STATEESTIMATORCONTROLLEROPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class StateEstimatorControllerOptions : public OptionContainer {
public:
	Option<std::string>* tPublisherTopic;
	Option<std::string>* tPluginLookupName;

	Option<std::string>* tTransformStampedTopic;

	Option<bool>* tPublishRosTransform;
	Option<bool>* tPublishRosTransformStamped;

    Option<std::string>* tTransformParentFrame;
    Option<std::string>* tTransformChildFrame;

	StateEstimatorControllerOptions();
};

}

#endif /* STATEESTIMATORCONTROLLEROPTIONS_HPP_ */
