/*
 * BehaviorControllerOptions.hpp
 *
 *  Created on: Nov 11, 2011
 *      Author: mriedel
 */

#ifndef BEHAVIORCONTROLLEROPTIONS_HPP_
#define BEHAVIORCONTROLLEROPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

#include <telekyb_base/Base/Singleton.hpp>

namespace TELEKYB_NAMESPACE {

class BehaviorControllerOptions : public OptionContainer, public Singleton<BehaviorControllerOptions> {
public:
	Option<double>* tInitialStateTimeout;
	Option<std::string>* tStateEstimationTopic;
	BehaviorControllerOptions();
};

}

#endif /* BEHAVIORCONTROLLEROPTIONS_HPP_ */
