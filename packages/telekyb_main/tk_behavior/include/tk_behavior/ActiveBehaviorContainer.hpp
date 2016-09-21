/*
 * ActiveBehaviorContainer.hpp
 *
 *  Created on: Mar 13, 2012
 *      Author: mriedel
 */

#ifndef ACTIVEBEHAVIORCONTAINER_HPP_
#define ACTIVEBEHAVIORCONTAINER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/enum.hpp>

#include <telekyb_base/Options.hpp>

#include <tk_state/StateEstimatorController.hpp>

#include <tk_behavior/Behavior.hpp>
#include <tk_behavior/BehaviorControllerOptions.hpp>

// boost
#include <boost/thread.hpp>

// ENUM
TELEKYB_ENUM(TransitionState,
		(Active)
		(Termination)
		(Creation)
)


// Forward declaration
class BehaviorController;

namespace TELEKYB_NAMESPACE
{

class ActiveBehaviorContainer {
protected:
	Behavior* activeBehaviorPtr; // Behavior Pointer.

	BehaviorControllerOptions& options;

	// StateEstimatorController Instance!
	StateEstimatorController& seController;

	// BehaviorController Instance for Callbacks
	BehaviorController& behaviorController;

	// Boost Threading stuff
	boost::thread behaviorChangeThread;
	boost::mutex behaviorChangeMutex;
	// Behavior ChangeRequestMutex. One cannot ask for BehaviorSwitch during Trajectory Callback
	boost::mutex behaviorChangeRequestMutex;

	// Variables from thread
	TransitionState transitionState;

	void switchToBehaviorThread(Behavior* newBehavior);

	// Interface for Switch from TKTrajectory.
	void switchToBehaviorInt(Behavior* newBehavior);

	bool checkNewBehavior(Behavior* newBehavior) const;

public:
	ActiveBehaviorContainer(Behavior* initialActiveBehavior);
	virtual ~ActiveBehaviorContainer();

	// Trajectory Step for active Behavior
	void trajectoryStep(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Switch to Behavior. External Request.
	bool switchToBehavior(Behavior* newBehavior);

	Behavior* getActive() const;

};

} // namespace

#endif /* ACTIVEBEHAVIORCONTAINER_HPP_ */

