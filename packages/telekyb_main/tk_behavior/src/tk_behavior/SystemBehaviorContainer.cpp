/*
 * SystemBehaviorContainer.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#include <tk_behavior/SystemBehaviorContainer.hpp>

namespace TELEKYB_NAMESPACE {

SystemBehaviorContainer::SystemBehaviorContainer()
{
	// Unneeded, but out of correctness.
	normalBrake = NULL;
	hover = NULL;
	ground = NULL;

	// Load SystemBehaviors
	loadSystemBehaviors();
}

SystemBehaviorContainer::~SystemBehaviorContainer()
{

}

void SystemBehaviorContainer::loadSystemBehaviors()
{
	// OK this is NOT done dynamically. Adding and modifying of SystemBehaviors is done here!

	// load
	normalBrake = static_cast<NormalBrake*>(loadBehavior( "tk_behavior/NormalBrake" ));
	hover = static_cast<Hover*>(loadBehavior( "tk_behavior/Hover" ));
	ground = static_cast<Ground*>(loadBehavior( "tk_behavior/Ground" ));

	emergencyLand = static_cast<EmergencyLand*>(loadBehavior( "tk_behavior/EmergencyLand" ));

	// load (not externally visible)
	TakeOff* takeOff = static_cast<TakeOff*>(loadBehavior( "tk_behavior/TakeOff" ));
	Land* land = static_cast<Land*>(loadBehavior( "tk_behavior/Land" ));

	// check MANDATORY!
	if (!(normalBrake && hover && ground && emergencyLand && takeOff && land)) {
		ROS_FATAL("An Error occured while trying to load essential System Behaviors!");
		//ROS_BREAK();
		ros::shutdown();
	}

	// init TODO?

	// set follow up behaviors
	normalBrake->setNextBehavior(hover);
	land->setNextBehavior(ground);
	emergencyLand->setNextBehavior(ground);
}

Behavior* SystemBehaviorContainer::loadBehavior(const std::string& behaviorClassName)
{
	if ( systemBehaviorMap.count(behaviorClassName) > 0 ) {
		ROS_ERROR_STREAM("Tried to load System Behavior with Name " << behaviorClassName << " twice!");
		return NULL;
	}

	Behavior* b = BehaviorContainer::loadBehavior(behaviorClassName);
	// if != NULL -> add
	if ( b ) {
		systemBehaviorMap[ behaviorClassName ] = b;
	}
	return b;
}

Hover* SystemBehaviorContainer::getHover() const
{
	return hover;
}
Ground* SystemBehaviorContainer::getGround() const
{
	return ground;
}
NormalBrake* SystemBehaviorContainer::getNormalBrake() const
{
	return normalBrake;
}

EmergencyLand* SystemBehaviorContainer::getEmergencyLand() const
{
	return emergencyLand;
}

Behavior* SystemBehaviorContainer::getBehaviorByName(const std::string& behaviorClassName) const {
	Behavior* b = NULL;
	if ( systemBehaviorMap.count(behaviorClassName) > 0 ) {
		b = systemBehaviorMap.at(behaviorClassName);
	}
	return b;
}


} /* namespace telekyb */
