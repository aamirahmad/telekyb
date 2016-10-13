/*
 * Behavior.hpp
 *
 *  Created on: Oct 28, 2011
 *      Author: mriedel
 */

#ifndef BEHAVIOR_HPP_
#define BEHAVIOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Messages.hpp>

//For OptionContainer
#include <telekyb_base/Options.hpp>

#include <string>
#include <ros/ros.h>

// BehaviorType
#include <tk_behavior/BehaviorType.hpp>

// Interface
#include <tk_behavior/BehaviorInterface.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

namespace TELEKYB_NAMESPACE
{

// forward declaration
class BehaviorController;

// Interface definition for Behaviors

// Every Behavior is an OptionContainer
class Behavior : public OptionContainer
{
private:
	// cannot be overwritten by Behavior implementations!
	BehaviorType type;

protected:
	Behavior(const std::string& name_, const BehaviorType& type_);

	BehaviorController& bController;
	// ROS
	ros::NodeHandle nodeHandle;

	// name <- set by BehaviorContainer, since it loads by name
	std::string name;
	//void setName(const std::string& name_);

	// is set to true after Behavior is Parameter-Initalized.
	// if Behavior has no Parameters, set this to true within initialize().
	bool parameterInitialized;

	// Follow up Behavior -> get's activated if trajectoryStep return false.
	Behavior* nextBehavior;

	// Interface for ROS
	BehaviorInterface* behaviorInterface;

public:
	// Pure Base Methods
	uint64_t getID() const;
	std::string getIDString() const;
	std::string getName() const;
	bool isParameterInitialized() const;
	void setParameterInitialized(bool parameterInitialized_);

	// get NodeHandle
	const ros::NodeHandle& getNodeHandle() const;

	// returns true if this behavior is the current active one
	bool isActive() const;

	// Next Behavior Control
	bool setNextBehavior(Behavior* nextBehavior_);
	void unsetNextBehavior();
	bool hasNextBehavior() const;
	Behavior* getNextBehavior() const;

	BehaviorType getType() const;

	// Method with old/current Behavior. This is called by the Controller and will call
	bool canTransitionTo(const Behavior& toBehavior) const;

	// called directly after Creation
	virtual void initialize() = 0;

	// called right before destruction
	virtual void destroy() = 0;

	// called right before Behavior becomes active. Get's the currentState once.
	virtual bool willBecomeActive(const TKState& currentState, const Behavior& previousBehavior) = 0;

	// called right after Behavior became active. Get's the currentState once.
	virtual void didBecomeActive(const TKState& currentState, const Behavior& previousBehavior) = 0;

	// called right before Behavior becomes inactive
	virtual void willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior) = 0;

	// called right after Behavior become inactive
	virtual void didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior) = 0;

	// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
	virtual void trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput) = 0;

    // called from active behavior container right bevor trajectoryStepActive
    virtual void setTrajectoryHeader(TKTrajectory& generatedTrajInput);

    // called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
	virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput) = 0;

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput) = 0;

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const = 0;

	// Destructor
	virtual ~Behavior();

	// Friend BehaviorContainer to setName
	//friend class BehaviorContainer;

	// static Helpers
	static Behavior* behaviorFromID(uint64_t behaviorID);
	static uint64_t behaviorToID(Behavior* b);
	static bool exists(Behavior *behavior);
	static bool exists(uint64_t behaviorID);
};


} // namepsace


#endif /* BEHAVIOR_HPP_ */
