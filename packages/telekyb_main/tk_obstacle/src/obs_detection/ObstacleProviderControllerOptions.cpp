/*
 * ObstacleProviderControllerOptions.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <obs_detection/ObstacleProviderControllerOptions.hpp>

namespace TELEKYB_NAMESPACE {

ObstacleProviderControllerOptions::ObstacleProviderControllerOptions()
	: OptionContainer("ObstacleProviderController")
{
	tObsSpinrate = addBoundsOption<double>("tObsSpinrate",
			"Specifies the rate at which the Controller collects Obstacles from Providers", 30.0, 0.0, 240.0, false, true);

	std::vector<std::string> defaultObstacleProviders;
	defaultObstacleProviders.push_back("tk_obstacle/SurroundingBox");
	//defaultObstacleProviders.push_back("tk_trajprocessor/XMModeCheck");

	tObstacleProviders = addOption("tObstacleProviders", "List of tObstacle Providers Plugins to load.",
			defaultObstacleProviders, false, true);
	tTKStateTopicName = addOption<std::string>("tTKStateTopicName", "Topic Name of Robots TKState",
			"undef", true, true);
	tObsPubTopicName = addOption<std::string>("tObsPubTopicName", "Topic Name of Obstacle Points",
			"ObstaclePoints", false, true);
	tInitialStateTimeout = addBoundsOption<double>("tInitialStateTimeout",
							"Specify the time to wait for the initial TKState Message.", 5.0, 0.0, 20.0, false, true);

}


} /* namespace telekyb */
