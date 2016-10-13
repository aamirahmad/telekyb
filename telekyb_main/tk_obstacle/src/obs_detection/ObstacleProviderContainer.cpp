/*
 * ObstacleProviderContainer.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <obs_detection/ObstacleProviderContainer.hpp>

#include <boost/foreach.hpp>

namespace TELEKYB_NAMESPACE {

ObstacleProviderContainer::ObstacleProviderContainer()
	: obstacleProviderLoader("tk_obstacle", std::string( TELEKYB_NAMESPACE_STRING ) + "::ObstacleProvider")
{

}

ObstacleProviderContainer::~ObstacleProviderContainer()
{
	// delete all Behaviors that are still loaded
	BOOST_FOREACH(ObstacleProvider *op, obstacleProviderInstances) {
		op->destroy();
		delete op;
	}
}

// should be const, but getDeclaredClasses is declared wrongly.
void ObstacleProviderContainer::getAvailableObstacleProviders(std::vector<std::string>& obstacleProviderClassNames)
{
	obstacleProviderClassNames = obstacleProviderLoader.getDeclaredClasses();
}

// returns 0 if fails
ObstacleProvider* ObstacleProviderContainer::loadObstacleProvider(const std::string& obstacleProviderClassName)
{
	ObstacleProvider *op = NULL;
	try {
		op = obstacleProviderLoader.createClassInstance(obstacleProviderClassName);

		// initialize
		op->initialize();

		// add local
		obstacleProviderInstances.insert(op);

		// success
		//ROS_INFO_STREAM("Successfully loaded Behavior: " << behaviorClassName << ", Instance: " << (long)b);
	} catch (pluginlib::PluginlibException &e) {
		ROS_ERROR_STREAM("ObstacleProvider Plugin " << obstacleProviderClassName << " failed to load. Message: " << e.what());
	}

	return op;
}

void ObstacleProviderContainer::unLoadObstacleProvider(ObstacleProvider* op)
{
	ROS_INFO_STREAM("Unloading Trajectory Module: " << op->getName());
	if(obstacleProviderInstances.erase(op)) {
		// Inform behavior
		op->destroy();
		// delete
		delete op;
	}
}

const std::set<ObstacleProvider*>& ObstacleProviderContainer::getLoadedObstacleProviders() const
{
	return obstacleProviderInstances;
}

} /* namespace telekyb */
