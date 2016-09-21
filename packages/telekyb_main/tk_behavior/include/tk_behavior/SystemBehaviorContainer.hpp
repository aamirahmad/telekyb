/*
 * SystemBehaviorContainer.hpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#ifndef SYSTEMBEHAVIORCONTAINER_HPP_
#define SYSTEMBEHAVIORCONTAINER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_behavior/BehaviorContainer.hpp>

// Ok it's a little bit confusing that behavior plugin path and include are different.
// I should rename this.
// ros_package name != include folder != behavior namespace :(
#include <behaviors/NormalBrake.hpp>
#include <behaviors/Hover.hpp>
#include <behaviors/Ground.hpp>
#include <behaviors/TakeOff.hpp>
#include <behaviors/Land.hpp>
#include <behaviors/EmergencyLand.hpp>

using namespace telekyb_behavior;

namespace TELEKYB_NAMESPACE {

class SystemBehaviorContainer : public BehaviorContainer {
protected:
	// map
	std::map<std::string, Behavior*> systemBehaviorMap;

	NormalBrake* normalBrake;
	Hover* hover;
	Ground* ground;
	EmergencyLand* emergencyLand;

	// These are never interfaced by the controller.
	//TakeOff* takeOff;
	//Land* land;

	// called by Constructor. Loads System Behaviors.
	void loadSystemBehaviors();

	// Overwrite to use Map
	Behavior* loadBehavior(const std::string& behaviorClassName);

public:
	SystemBehaviorContainer();
	virtual ~SystemBehaviorContainer();

	// Accessor System Behaviors
	NormalBrake* getNormalBrake() const;
	Hover* getHover() const;
	Ground* getGround() const;
	EmergencyLand* getEmergencyLand() const;
	//TakeOff* getTakeOff() const;
	//Land* getLand() const;

	Behavior* getBehaviorByName(const std::string& behaviorClassName) const;
};

} /* namespace telekyb */
#endif /* SYSTEMBEHAVIORCONTAINER_HPP_ */
