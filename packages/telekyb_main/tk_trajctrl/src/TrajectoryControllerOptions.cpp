/*
 * TrajectoryControllerOptions.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include <tk_trajctrl/TrajectoryControllerOptions.hpp>

namespace TELEKYB_NAMESPACE {

TrajectoryControllerOptions::TrajectoryControllerOptions()
	: OptionContainer("TrajectoryController")
{
	tPluginLookupName = addOption<std::string>("tPluginLookupName",
			"Specifies the Trajectory Tracker Plugin.", "trajectory_trackers_plugin::StandardTrajectoryTracker", false, true);



	tDoMassEstimation = addOption<bool>("tDoMassEstimation",
			"IMPORTANT: This represents the CURRENT State of Mass Estimation. e.g. it is disabled on ground. Can be toggled by behaviors"
			, false, false, false);

	tDoInertiaMatrixEstimation = addOption<bool>("tDoInertiaMatrixEstimation",
				"IMPORTANT: This represents the CURRENT State of Inertia Matrix Estimation. Can be toggled by behaviors"
				, false, false, false);


	tStateEstimationTopic = addOption<std::string>("tStateEstimationTopic",
			"Specifies the topic where the state estimation is being published (by default this is the topic where the TK state estimator is publishing).",
			"none", false, true);


}


}
