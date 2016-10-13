/*
 * TrajectoryProcessorControllerOptions.cpp
 *
 *  Created on: Dec 13, 2011
 *      Author: mriedel
 */

#include <tk_trajprocessor/TrajectoryProcessorControllerOptions.hpp>

namespace TELEKYB_NAMESPACE {

TrajectoryProcessorControllerOptions::TrajectoryProcessorControllerOptions()
	: OptionContainer("TrajectoryProcessorController")
{
	// Init Options
	tInitialStateTimeout = addBoundsOption<double>("tInitialStateTimeout",
							"Specify the time to wait for the initial TKState Message.", 2.0, 0.0, 60.0, false, true);
	std::vector<std::string> defaultTrajectoryModules;
	defaultTrajectoryModules.push_back("tk_trajprocessor/PMPositionError");
	defaultTrajectoryModules.push_back("tk_trajprocessor/XMModeCheck");

	tTrajectoryModules = addOption("tTrajectoryModules", "List of Trajectory Modules Plugins to load.",
			defaultTrajectoryModules, false, true);
	
	tStateEstimationTopic = addOption<std::string>("tStateEstimationTopic",
			"Specifies the topic where the state estimation is being published (by default this is the topic where the TK state estimator is publishing).",
			"none", false, true);
}


} /* namespace telekyb */
