/*
 * BehaviorController.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#include <telekyb_interface/BehaviorController.hpp>

#include <telekyb_srvs/StringArrayOutput.h>
#include <telekyb_srvs/BehaviorInput.h>
#include <telekyb_srvs/BehaviorOutput.h>
#include <telekyb_srvs/BehaviorInputOutput.h>


namespace TELEKYB_INTERFACE_NAMESPACE {

BehaviorController::BehaviorController(const std::string& behaviorHandleNamespace, OptionController* optionController_)
	: behaviorControllerNodeHandle(behaviorHandleNamespace), optionController(optionController_), listener(NULL)
{
	ROS_INFO_STREAM("Created BehaviorController Nodehandle: " << behaviorHandleNamespace);

	// Initial active Behavior
	activeBehavior = getActiveBehavior();

	// Subscribe
	activeBehaviorSub = behaviorControllerNodeHandle.subscribe(BEHAVIOR_BEHAVIORCHANGETOPIC, 10, &BehaviorController::activeBehaviorCallback ,this);
}

BehaviorController::~BehaviorController()
{

}

const ros::NodeHandle& BehaviorController::getNodeHandle() const
{
	return behaviorControllerNodeHandle;
}

OptionController* BehaviorController::getOptionController() const
{
	return optionController;
}


Behavior BehaviorController::getSystemBehavior(const std::string& behaviorName)
{
	ros::ServiceClient client = behaviorControllerNodeHandle.serviceClient<telekyb_srvs::BehaviorInputOutput>(BEHAVIOR_GETSYSTEMBEHAVIOR);
	telekyb_srvs::BehaviorInputOutput service;
	service.request.behaviorName = behaviorName;
	if (! client.call(service) ) {
        ROS_ERROR_STREAM("BehaviorController.cpp :: Failed to call: " << client.getService() << "! Returning NULL-Behavior.");
		return Behavior();
	}

	return Behavior(service.response.behaviorID, service.response.behaviorName, this);
}
// Get Available Behaviors.
bool BehaviorController::getAvailableBehaviors(std::vector<std::string>& behaviorNames)
{
	ros::ServiceClient client = behaviorControllerNodeHandle.serviceClient<telekyb_srvs::StringArrayOutput>(BEHAVIOR_GETAVAILABLEBEHAVIORS);
	telekyb_srvs::StringArrayOutput service;
	if (! client.call(service) ) {
        ROS_ERROR_STREAM("BehaviorController.cpp :: Failed to call: " << client.getService());
		return false;
	}

	behaviorNames = service.response.output;
	return true;
}

Behavior BehaviorController::loadBehavior(const std::string& behaviorName)
{
	ros::ServiceClient client = behaviorControllerNodeHandle.serviceClient<telekyb_srvs::BehaviorInputOutput>(BEHAVIOR_LOADBEHAVIOR);
	telekyb_srvs::BehaviorInputOutput service;
	service.request.behaviorName = behaviorName;

	if (! client.call(service) ) {
        ROS_ERROR_STREAM("BehaviorController.cpp :: Failed to call: " << client.getService() << "! Returning NULL-Behavior.");
		return Behavior();
	}

	return Behavior(service.response.behaviorID, service.response.behaviorName, this);
}
bool BehaviorController::unloadBehavior(Behavior& behavior)
{
	ros::ServiceClient client = behaviorControllerNodeHandle.serviceClient<telekyb_srvs::BehaviorInput>(BEHAVIOR_UNLOADBEHAVIOR);
	telekyb_srvs::BehaviorInput service;
	service.request.behaviorID = behavior.getBehaviorID();
	if (! client.call(service) ) {
        ROS_ERROR_STREAM("BehaviorController.cpp :: Failed to call: " << client.getService());
		return false;
	}

	// setNUll
	behavior.setNull();

	return true;
}

// Switch
bool BehaviorController::switchBehavior(const Behavior& behavior)
{
	ros::ServiceClient client = behaviorControllerNodeHandle.serviceClient<telekyb_srvs::BehaviorInput>(BEHAVIOR_SWITCHBEHAVIOR);
	telekyb_srvs::BehaviorInput service;
	service.request.behaviorID = behavior.getBehaviorID();
	if (! client.call(service) ) {
        ROS_ERROR_STREAM("BehaviorController.cpp :: Failed to call: " << client.getService());
		return false;
	}
	return true;
}

Behavior BehaviorController::getActiveBehavior()
{
	ros::ServiceClient client = behaviorControllerNodeHandle.serviceClient<telekyb_srvs::BehaviorOutput>(BEHAVIOR_GETACTIVEBEHAVIOR);
	telekyb_srvs::BehaviorOutput service;
	if (! client.call(service) ) {
        ROS_ERROR_STREAM("BehaviorController.cpp :: Failed to call: " << client.getService() << "! Returning NULL-Behavior.");
		return Behavior();
	}

	return Behavior(service.response.behaviorID, service.response.behaviorName, this);
}

std::string BehaviorController::getActiveBehaviorName(){
    ros::ServiceClient client = behaviorControllerNodeHandle.serviceClient<telekyb_srvs::BehaviorOutput>(BEHAVIOR_GETACTIVEBEHAVIOR);
    telekyb_srvs::BehaviorOutput service;
    if (! client.call(service) ) {
        ROS_ERROR_STREAM("BehaviorController.cpp :: Failed to call: " << client.getService() << "! Returning NULL-Behavior.");
        return "noNamefound";
    }

    return service.response.behaviorName;

}

Behavior* BehaviorController::getActiveBehaviorPointer()
{
	return &activeBehavior;
}

const Behavior& BehaviorController::getActiveBehaviorReference() const
{
	return activeBehavior;
}

void BehaviorController::activeBehaviorCallback(const telekyb_msgs::Behavior::ConstPtr& msg)
{
	ROS_INFO_STREAM("Active Behavior was changed to: " << msg->behaviorName);
	activeBehavior.behaviorID = msg->behaviorID;
	activeBehavior.behaviorName = msg->behaviorName;

	if (listener) {
		listener->activeBehaviorChanged(activeBehavior);
	}
}

void BehaviorController::setActiveBehaviorListener(ActiveBehaviorListener* listener_)
{
	listener = listener_;
}

}
