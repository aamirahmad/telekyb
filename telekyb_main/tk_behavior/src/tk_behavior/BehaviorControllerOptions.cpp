/*
 * BehaviorControllerOptions.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: mriedel
 */

#include <tk_behavior/BehaviorControllerOptions.hpp>

namespace TELEKYB_NAMESPACE {

//template<> BehaviorControllerOptions* Singleton<BehaviorControllerOptions>::instance = NULL;

BehaviorControllerOptions::BehaviorControllerOptions()
	: OptionContainer("BehaviorController")
{
	// Init Options
	tInitialStateTimeout = addBoundsOption<double>("tInitialStateTimeout",
							"Specify the time to wait for the initial TKState Message.", 2.0, 0.0, 60.0, false, true);
	tStateEstimationTopic = addOption<std::string>("tStateEstimationTopic",
			"Specifies the topic where the state estimation is being published (by default this is the topic where the TK state estimator is publishing).",
			"none", false, true);
}


}
