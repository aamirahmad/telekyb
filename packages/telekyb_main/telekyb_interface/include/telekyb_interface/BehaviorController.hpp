 /*
 * BehaviorController.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#ifndef INTERFACE_BEHAVIORCONTROLLER_HPP_
#define INTERFACE_BEHAVIORCONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <ros/ros.h>

#include <telekyb_interface/Behavior.hpp>
#include <telekyb_interface/OptionController.hpp>

#include <telekyb_msgs/Behavior.h>

namespace TELEKYB_INTERFACE_NAMESPACE {

// ActiveBehaviorListener

class ActiveBehaviorListener {
public:
	virtual ~ActiveBehaviorListener() {};
	virtual void activeBehaviorChanged(Behavior newActiveBehavior) = 0;
};

class BehaviorController {
protected:
	ros::NodeHandle behaviorControllerNodeHandle;
	// OptionController
	OptionController* optionController;

	// active Behavior
	Behavior activeBehavior;

	ros::Subscriber activeBehaviorSub;

	// get Active
	Behavior getActiveBehavior();

	// Listener // only one at ATM
	ActiveBehaviorListener* listener;

public:
	BehaviorController(const std::string& behaviorHandleNamespace, OptionController* optionController_);
	virtual ~BehaviorController();

	// return Nodehandle
	const ros::NodeHandle& getNodeHandle() const;

	// return OptionController
	OptionController* getOptionController() const;

	// System Behaviors are only loaded Once and can be referenced by Name.
	Behavior getSystemBehavior(const std::string& behaviorName);
	// Get Available Behaviors.
	bool getAvailableBehaviors(std::vector<std::string>& behaviorNames);
	Behavior loadBehavior(const std::string& behaviorName);
	bool unloadBehavior(Behavior& behavior);

	// Switch
	bool switchBehavior(const Behavior& behavior);

	// Current Behavior Pointer -> updates live
	Behavior* getActiveBehaviorPointer();
    // retrun the name of the active behavior via a service call
    std::string getActiveBehaviorName();
	const Behavior& getActiveBehaviorReference() const;

	// active Behavior Callback
	void activeBehaviorCallback(const telekyb_msgs::Behavior::ConstPtr& msg);

	// setActiveBehaviorListener;
	void setActiveBehaviorListener(ActiveBehaviorListener* listener_);
};

}

#endif /* BEHAVIORCONTROLLER_HPP_ */
