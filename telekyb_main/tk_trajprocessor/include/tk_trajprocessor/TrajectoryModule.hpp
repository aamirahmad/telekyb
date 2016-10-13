/*
 * TrajectoryModule.hpp
 *
 *  Created on: Dec 13, 2011
 *      Author: mriedel
 */

#ifndef TRAJECTORYMODULE_HPP_
#define TRAJECTORYMODULE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_trajprocessor/TrajectoryProcessorDefines.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

#include <telekyb_base/Messages.hpp>

namespace TELEKYB_NAMESPACE {

class TrajectoryProcessorController;

class TrajectoryModule {
private:
	// set individual Module active or inactive
	bool active;

protected:
	TrajectoryProcessorController& tpController;

	// pluginName
	std::string name;
	// type
	TrajModulePosType type;
	// priority 0-> you don't care-> will be added in the middle
	// priority < 0 the more negative it will be placed in the beginning
	// priority > 0 the more positive it will be placed in the end.
	// PRIORITIES SHOULD BE UNIQUE! OTHERWISE LOADING ORDER DETERMINES POSITION!
	int priority;


	TrajectoryModule(const std::string& name_, TrajModulePosType type_, int priority_);

public:
	virtual ~TrajectoryModule();
	TrajModulePosType getType() const;
	std::string getName() const;
	int getPriority() const;

	bool isActive() const;
	void setActive();
	void setInactive();


	// called directly after Creation
	virtual void initialize() = 0;

	// called right before destruction
	virtual void destroy() = 0;

	// set back to intial conditions
	virtual void willTurnActive() = 0;

	// called after turning inactive
	virtual void didTurnInactive() = 0;

	// Interface Functions
	// true -> proceed normally
	// false -> do not check other Trajectory Modules in List. (e.g. BehaviorChange etc...)
	virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& trajInput) = 0;
};

} /* namespace telekyb */
#endif /* TRAJECTORYMODULE_HPP_ */
