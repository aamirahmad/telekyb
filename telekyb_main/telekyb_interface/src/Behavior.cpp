/*
 * Behavior.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#include <telekyb_interface/Behavior.hpp>

#include <boost/lexical_cast.hpp>
using boost::lexical_cast;

// ROS Services
#include <telekyb_srvs/BehaviorInput.h>
#include <telekyb_srvs/BehaviorOutput.h>
#include <telekyb_srvs/BoolInput.h>


#include <telekyb_interface/BehaviorController.hpp>

namespace TELEKYB_INTERFACE_NAMESPACE {

Behavior::Behavior()
{
	setNull();
}

Behavior::Behavior(uint64_t behaviorID_, const std::string& behaviorName_, BehaviorController* behaviorController_)
	: behaviorID(behaviorID_),
	  behaviorName(behaviorName_),
	  behaviorController(behaviorController_),
	  behaviorNodeHandle(behaviorController->getNodeHandle(), lexical_cast<std::string>(behaviorID_))
{

}

Behavior::~Behavior()
{

}

void Behavior::setNull()
{
	behaviorID = (uint64_t)NULL;
	behaviorName = "";
	// nodeHandle is default.
}

uint64_t Behavior::getBehaviorID() const
{
	return behaviorID;
}

std::string Behavior::getBehaviorName() const
{
	return behaviorName;
}

void Behavior::setNextBehavior(const Behavior& behavior)
{
    ros::ServiceClient client = behaviorNodeHandle.serviceClient<telekyb_srvs::BehaviorInput>(BEHAVIOR_SETNEXTBEHAVIOR);
    telekyb_srvs::BehaviorInput service;
	service.request.behaviorID = behavior.getBehaviorID();
	service.request.behaviorName = behavior.getBehaviorName();
	if (! client.call(service) ) {
		ROS_ERROR_STREAM("Failed to call: " << client.getService());
	}
}

Behavior Behavior::getNextBehavior()
{
    ros::ServiceClient client = behaviorNodeHandle.serviceClient<telekyb_srvs::BehaviorOutput>(BEHAVIOR_GETNEXTBEHAVIOR);
    telekyb_srvs::BehaviorOutput service;
	if (! client.call(service) ) {
        ROS_ERROR_STREAM("Behavior.cpp :: Failed to call: " << client.getService() << "! Returning NULL-Behavior.");
		return Behavior();
	}
	return Behavior(service.response.behaviorID, service.response.behaviorName, behaviorController);
}

void Behavior::setParameterInitialized(bool initialized_)
{
    ros::ServiceClient client = behaviorNodeHandle.serviceClient<telekyb_srvs::BoolInput>(BEHAVIOR_SETPARAMINIT);
    telekyb_srvs::BoolInput service;
	service.request.input = initialized_;
	if (! client.call(service) ) {
		ROS_ERROR_STREAM("Failed to call: " << client.getService());
	}
}

// get OptionContainer
OptionContainer Behavior::getOptionContainer()
{
	// returns the OptionContainer associated with the Behavior
	return OptionContainer(behaviorController->getOptionController(), behaviorName + "/" + lexical_cast<std::string>(behaviorID));

}

bool Behavior::isNull() const
{
	return (behaviorID == (uint64_t)NULL);
}

// Operators
bool operator==(Behavior& lhs, Behavior& rhs)
{
	//ROS_INFO("called Behavior == Behavior");
	// Temp implementation // only IDs matter
	return (lhs.behaviorID == rhs.behaviorID);
}
bool operator!=(Behavior& lhs, Behavior& rhs)
{
	return !(lhs == rhs);
}

} /* namespace telekyb_interface */
