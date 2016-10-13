/*
 * BehaviorController.hpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#ifndef BEHAVIORCONTROLLER_HPP_
#define BEHAVIORCONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

// BehaviorSwitcher Interface
#include <telekyb_defines/ClassInterfaces.hpp>

#include <telekyb_base/Messages.hpp>

#include <tk_behavior/BehaviorControllerInterface.hpp>
#include <tk_behavior/ActiveBehaviorContainer.hpp>

// BehaviorContainers. Systemcontains preloaded Behaviors
#include <tk_behavior/SystemBehaviorContainer.hpp>

#include <tk_behavior/Behavior.hpp>

#include <ros/ros.h>

namespace TELEKYB_NAMESPACE {

//forward declaration
class TrajectoryProcessorController;

class BehaviorController : public BehaviorSwitcher {
private:
	// Singleton Stuff
	static BehaviorController* instance;

	BehaviorController();
	virtual ~BehaviorController();

	BehaviorController(const BehaviorController &);
	BehaviorController& operator=(const BehaviorController &);

protected:
	
	// BehaviorNodeHandle
	ros::NodeHandle behaviorNodeHandle;

	// State Subscription
	ros::Subscriber tStateSub;

	// BehaviorController Parts
	BehaviorControllerOptions options;
	SystemBehaviorContainer* systemBehaviorContainer;
	BehaviorControllerInterface* bcInterface;


	// active Behavior
	ActiveBehaviorContainer* activeBehavior;
//	Behavior* activeBehavior;
//	BehaviorSwitch behaviorSwitcher; // responsible for the Switch.

	// Last TKTrajInput
	TKTrajectory lastInput;

	// where to send the TKTrajInput
	TrajectoryProcessorController& trajProcCtrl;


	// initialize. This does further setup AFTER the object has been created.
	// This is needed, since Objects that reference the Behavior Controller can only be created after it returns from the constuctor
	// (Singleton Issue).
	//** This is like a Constructor. It's called by the Singleton creator DIRECTLY AFTER the actual constructor. **/
	void initialize();

	// Callback
	void tkStateCB(const telekyb_msgs::TKState::ConstPtr& tkStateMsg);

public:

	const ros::NodeHandle& getBehaviorNodeHandle() const;
	Behavior* getActiveBehavior() const;

	// Switch Behavior
	bool switchToBehavior(Behavior* newBehavior);

	const SystemBehaviorContainer& getSystemBehaviorContainer() const;

	bool switchToNormalBrake();
	bool switchToEmergencyLand();

	void activeBehaviorChanged();

	// Singleton Stuff
	static BehaviorController& Instance();
	static const BehaviorController* InstancePtr();
	static bool HasInstance();
	static void ShutDownInstance();
};

}

#endif /* BEHAVIORCONTROLLER_HPP_ */
