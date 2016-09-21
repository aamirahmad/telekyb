/*
 * TrajectoryModule.cpp
 *
 *  Created on: Dec 13, 2011
 *      Author: mriedel
 */

#include <tk_trajprocessor/TrajectoryModule.hpp>

#include <tk_trajprocessor/TrajectoryProcessorController.hpp>

namespace TELEKYB_NAMESPACE {

TrajectoryModule::TrajectoryModule(const std::string& name_, TrajModulePosType type_, int priority_)
	: active(false),
	  tpController(TrajectoryProcessorController::Instance()),
	  name(name_), type(type_),  priority(priority_)
{

}

TrajectoryModule::~TrajectoryModule()
{
//	printf("Destroying %s!\n", getName().c_str());
}

TrajModulePosType TrajectoryModule::getType() const
{
	return type;
}

std::string TrajectoryModule::getName() const
{
	return name;
}
int TrajectoryModule::getPriority() const
{
	return priority;
}

bool TrajectoryModule::isActive() const
{
	return active;
}
void TrajectoryModule::setActive()
{
	if (active) { return; }

	// set active
	willTurnActive();
	active = true;
}

void TrajectoryModule::setInactive()
{
	if (!active) { return; }

	// set inactive
	active = false;
	didTurnInactive();
}

} /* namespace telekyb */
