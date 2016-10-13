/*
 * BehaviorSwitch.cpp
 *
 *  Created on: Mar 13, 2012
 *      Author: mriedel
 */

#include <tk_behavior/ActiveBehaviorContainer.hpp>
#include <tk_behavior/BehaviorContainer.hpp>

#include <tk_behavior/BehaviorController.hpp>

namespace TELEKYB_NAMESPACE
{

ActiveBehaviorContainer::ActiveBehaviorContainer(Behavior* initialActiveBehavior)
	: activeBehaviorPtr( initialActiveBehavior ),
	  options( BehaviorControllerOptions::Instance() ),
	  seController( StateEstimatorController::Instance() ),
	  behaviorController( BehaviorController::Instance() )
{


	// did you get at least one state?
	if (! seController.waitForFirstState( Time(options.tInitialStateTimeout->getValue()) ) ) {
		// did not receive State
        ROS_ERROR("BehaviorController: Did not receive initial TKState within Timeout! Using uninitialized! This is dangerous!");
	}

	// this should block till state becomes available // old Behavior is itself;
	activeBehaviorPtr->willBecomeActive( seController.getLastState(), *activeBehaviorPtr);

	// Post active
	activeBehaviorPtr->didBecomeActive( seController.getLastState(), *activeBehaviorPtr);

	// State:
	transitionState = TransitionState::Active;

}

ActiveBehaviorContainer::~ActiveBehaviorContainer() {
	// TODO Auto-generated destructor stub
}

// Callbacks only come in after constructor is finished (Because of subscription point)
void ActiveBehaviorContainer::trajectoryStep(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Cannot Ask for Behaviorchange during this CB. Additionally prevents from calling this in parallel.
	boost::mutex::scoped_lock lock(behaviorChangeRequestMutex);

	// Currently changing Behavior?
	if (behaviorChangeMutex.try_lock()) {

		// no Change Thread running
		if (activeBehaviorPtr->isValid(currentState)) {
			// Active
            activeBehaviorPtr->setTrajectoryHeader(generatedTrajInput);
			activeBehaviorPtr->trajectoryStepActive(currentState, generatedTrajInput);

			// unlock again
			behaviorChangeMutex.unlock();
		} else {
			// InValid -> active in Termination
			activeBehaviorPtr->trajectoryStepTermination(currentState, generatedTrajInput);
			transitionState = TransitionState::Termination;

			// Start Switch Thread...
			Behavior* next = NULL; // NULL means Normalbrake!
			if (activeBehaviorPtr->hasNextBehavior()) {
				next = activeBehaviorPtr->getNextBehavior();
			}

			// null or not ok? -> Ground or Normalbrake
			if (!next || !checkNewBehavior(next)) {
				if (activeBehaviorPtr->getType() == BehaviorType::Ground) {
					next =  behaviorController.getSystemBehaviorContainer().getGround();
				} else {
					// Liftoff, Land and Air changeerrors
					next = behaviorController.getSystemBehaviorContainer().getNormalBrake();
				}
			}


			// change Behavior
			behaviorChangeThread = boost::thread(&ActiveBehaviorContainer::switchToBehaviorThread, this, next);

			// Unlock is done by thread
		}

	} else {

		// Change Thread Running.
		//ROS_ERROR("Currently Changing Behavior");
		// We have to check what state we are in

		switch (transitionState.index()) {
		case TransitionState::Active:
			if (activeBehaviorPtr->isValid(currentState)) {
				activeBehaviorPtr->trajectoryStepActive(currentState, generatedTrajInput);
			} else {
				// New Behavior is already invalid! But change still in Progress. Call Termination Trajectory Step of
				// this Behavior until previous Change has finished -> Then it automatically causes Change request.
				activeBehaviorPtr->trajectoryStepTermination(currentState, generatedTrajInput);
			}
			break;
		case TransitionState::Termination:
			activeBehaviorPtr->trajectoryStepTermination(currentState, generatedTrajInput);
			break;
		case TransitionState::Creation:
			activeBehaviorPtr->trajectoryStepCreation(currentState, generatedTrajInput);
			break;
		default:
			// never happens.
			break;
		}
	}
}

bool ActiveBehaviorContainer::checkNewBehavior(Behavior* newBehavior) const
{
	// newBehavior must not be the active one
	if (activeBehaviorPtr == newBehavior) {
        ROS_ERROR_STREAM_THROTTLE(1,"Prevented Behavior Switch of " << activeBehaviorPtr->getName() << ", because the Behavior is already active!");
		return false;
	}

	// Valid Behavior Check
	if (!BehaviorContainer::behaviorInstanceExists(newBehavior)) {
        ROS_ERROR_STREAM_THROTTLE(1,"Cannot switch Behavior to ID " << (uint64_t)newBehavior << ", because it does not exist.");
		return false;
	}

	// is Allowed Transition?
	if (!activeBehaviorPtr->canTransitionTo(*newBehavior)) {
        ROS_ERROR_STREAM_THROTTLE(1,"Transition from "
				<< activeBehaviorPtr->getName() << " (" <<  activeBehaviorPtr->getType().str() <<")"
				<< " to "
				<< newBehavior->getName() << " (" <<  newBehavior->getType().str() <<")"
				<< " is an unallowed transition.");
		return false;
	}

	// Check for Parameter Initalization
	if (!newBehavior->isParameterInitialized() ) {
        ROS_ERROR_STREAM_THROTTLE(1,"Cannot switch to Behavior " << newBehavior->getName() <<
				" ID: "<< (uint64_t)newBehavior << ", because it is not Parameter Initalized!");
		return false;
	}

	return true;
}

// Switch to Behavior
bool ActiveBehaviorContainer::switchToBehavior(Behavior* newBehavior)
{
	// Only do switch to Behavior if not in TrajectoryStep Callback.
	boost::mutex::scoped_lock lock(behaviorChangeRequestMutex);

	// Mutex try_lock.?
	if (!behaviorChangeMutex.try_lock()) {
        ROS_ERROR_STREAM_THROTTLE(1,"Prevented Behavior Switch of " << activeBehaviorPtr->getName() << ", because a Switch is currently in progess!");
		return false;
	}

	if (!checkNewBehavior(newBehavior)) {
		// transition not ok
		behaviorChangeMutex.unlock();
		return false;
	}

	transitionState = TransitionState::Termination;

	// has lock - start change Thread
	behaviorChangeThread = boost::thread(&ActiveBehaviorContainer::switchToBehaviorThread, this, newBehavior);

	return true;
}

void ActiveBehaviorContainer::switchToBehaviorThread(Behavior* newBehavior)
{
	// Inform Behavior about the switch. If this returns false. The behavior prevents the switch!
	if (!newBehavior->willBecomeActive( seController.getLastState(), *activeBehaviorPtr ) ) {
        ROS_ERROR_STREAM_THROTTLE(1,"Cannot switch to Behavior " << newBehavior->getName() <<
				" ID: "<< newBehavior->getIDString() << ", because it denied the switch.");


		// Go to Ground or NormalBrake
		if (activeBehaviorPtr->getType() == BehaviorType::Ground) {
			newBehavior =  behaviorController.getSystemBehaviorContainer().getGround();
		} else {
			// Liftoff, Land and Air changeerrors
			newBehavior = behaviorController.getSystemBehaviorContainer().getNormalBrake();
		}

		// if we are already in Normalbrake or Ground. exit.
		if (newBehavior == activeBehaviorPtr) {
			behaviorChangeMutex.unlock();
			return;
		}


		//** Note: These Behaviors must always returns true.
		if (!newBehavior->willBecomeActive( seController.getLastState(), *activeBehaviorPtr )) {
			ROS_FATAL("SytemBehavior %s willBecomeActive returned false. THIS MUST NOT HAPPEN!", newBehavior->getName().c_str());
		}
	}

	// Inform old
	activeBehaviorPtr->willBecomeInActive( seController.getLastState(), *newBehavior );

    ROS_INFO_STREAM_THROTTLE(1,"Behavior: Switching from " << activeBehaviorPtr->getName() << " to " << newBehavior->getName());

	// set to new. TURNING POINT
	// ------------------------- //
	Behavior* oldBehavior = activeBehaviorPtr;
	activeBehaviorPtr = newBehavior;
	// ------------------------- //

	transitionState = TransitionState::Creation;
	// change was successful.


	// notify about the change
	behaviorController.activeBehaviorChanged();
	// Post-Switch Callbacks
	activeBehaviorPtr->didBecomeActive( seController.getLastState(), *oldBehavior );

	transitionState = TransitionState::Active;

	oldBehavior->didBecomeInActive( seController.getLastState(), *activeBehaviorPtr );

	// unlock
	behaviorChangeMutex.unlock();
}

Behavior* ActiveBehaviorContainer::getActive() const
{
	return activeBehaviorPtr;
}


} // namespace
