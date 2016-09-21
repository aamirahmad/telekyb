/*
 * Behavior.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#ifndef INTERFACE_BEHAVIOR_HPP_
#define INTERFACE_BEHAVIOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_interface/OptionContainer.hpp>

#include <ros/ros.h>

namespace TELEKYB_INTERFACE_NAMESPACE {

// forward declaration
class BehaviorController;

// Behavior with behaviorID == 0 are invalid!

class Behavior {
private:
	// Null-Init
	Behavior(uint64_t behaviorID_, const std::string& behaviorName_, BehaviorController* behaviorController_);

	// set Null
	void setNull();

protected:
	uint64_t behaviorID;
	std::string behaviorName;

	// BehaviorNodeHandle
	BehaviorController* behaviorController;

	ros::NodeHandle behaviorNodeHandle;

public:
	Behavior(); // can always NULL Init
	virtual ~Behavior();

	void setNextBehavior(const Behavior& behavior);
	Behavior getNextBehavior();

	// option can only be loaded if set to true!
	void setParameterInitialized(bool initialized_);

	// get OptionContainer
	OptionContainer getOptionContainer();

	uint64_t getBehaviorID() const;
	std::string getBehaviorName() const;

	bool isNull() const;

	friend class BehaviorController;

	// Operators
	friend bool operator==(Behavior& lhs, Behavior& rhs);
	friend bool operator!=(Behavior& lhs, Behavior& rhs);

};

} /* namespace telekyb_interface */
#endif /* BEHAVIOR_HPP_ */
