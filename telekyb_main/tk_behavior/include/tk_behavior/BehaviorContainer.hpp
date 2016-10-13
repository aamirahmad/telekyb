/*
 * BehaviorContainer.hpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#ifndef BEHAVIORCONTAINER_HPP_
#define BEHAVIORCONTAINER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_behavior/Behavior.hpp>
#include <pluginlib/class_loader.h>

//#include <tk_behavior/BehaviorContainerOptions.hpp>

// stl
#include <set>
#include <string>


namespace TELEKYB_NAMESPACE {

class BehaviorContainer {
protected:
	// Options
	//BehaviorContainerOptions options;

	// static ClassLoader, available to all Containers
	pluginlib::ClassLoader<Behavior> behaviorLoader;
	// Behaviorset. Contains all loaded Behaviors from all Containers!
	static std::set<Behavior*> globalBehaviorInstances;

	std::set<Behavior*> behaviorInstances; // beware there can be several Instances of the same behavior

public:
	BehaviorContainer();
	virtual ~BehaviorContainer();

	// should be const, but getDeclaredClasses is declared wrongly.
	void getAvailableBehaviors(std::vector<std::string>& behaviorClassNames);

	// returns 0 if fails
	Behavior* loadBehavior(const std::string& behaviorClassName);
	bool unloadBehavior(Behavior* b);

	static bool behaviorInstanceExists(Behavior* instance);
};

}

#endif /* BEHAVIORCONTAINER_HPP_ */
