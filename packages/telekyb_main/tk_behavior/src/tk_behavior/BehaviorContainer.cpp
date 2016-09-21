/*
 * BehaviorContainer.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include <tk_behavior/BehaviorContainer.hpp>

#include <boost/foreach.hpp>

namespace TELEKYB_NAMESPACE {

// Static Initialization
//pluginlib::ClassLoader<Behavior> BehaviorContainer::behaviorLoader(
//		"tk_behavior", std::string( TELEKYB_NAMESPACE_STRING ) + "::Behavior");
std::set<Behavior*> BehaviorContainer::globalBehaviorInstances;


BehaviorContainer::BehaviorContainer()
	: behaviorLoader("tk_behavior", std::string( TELEKYB_NAMESPACE_STRING ) + "::Behavior")
{

}

BehaviorContainer::~BehaviorContainer()
{
	// delete all Behaviors that are still loaded
	BOOST_FOREACH(Behavior *b, behaviorInstances) {
		b->destroy();
		delete b;
	}
}

Behavior* BehaviorContainer::loadBehavior(const std::string& behaviorClassName)
{
	Behavior *b = NULL;
	try {
		b = behaviorLoader.createClassInstance(behaviorClassName);

		// inform behavior about its name.
		//b->setName(behaviorClassName);
		// initialize
		b->initialize();

		// add local
		behaviorInstances.insert(b);
		// add global
		globalBehaviorInstances.insert(b);

		// success
		//ROS_INFO_STREAM("Successfully loaded Behavior: " << behaviorClassName << ", Instance: " << (long)b);
	} catch (pluginlib::PluginlibException &e) {
		ROS_ERROR_STREAM("Behavior Plugin " << behaviorClassName << " failed to load. Message: " << e.what());
	}

	return b;
}

bool BehaviorContainer::unloadBehavior(Behavior* b)
{
	if(behaviorInstances.erase(b)) {
		// erase from global
		globalBehaviorInstances.erase(b);
		// Inform behavior
		b->destroy();
		// delete
		delete b;
		ROS_INFO_STREAM("Successfully unloaded Behavior with ID: " << (long)b);
		return true;
	} else {
		// was not element of BehaviorContainer
		ROS_ERROR_STREAM("Could not delete Behavior with ID: " << (long)b << ". Did not exist!");
		return false;
	}
}

void BehaviorContainer::getAvailableBehaviors(std::vector<std::string>& behaviorClassNames)
{
	behaviorClassNames = behaviorLoader.getDeclaredClasses();
}

bool BehaviorContainer::behaviorInstanceExists(Behavior* instance)
{
	return globalBehaviorInstances.count(instance) != 0;
}





}
