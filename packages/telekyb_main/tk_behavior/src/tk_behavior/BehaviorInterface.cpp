/*
 * BehaviorInterface.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: mriedel
 */

#include <tk_behavior/BehaviorInterface.hpp>

#include <tk_behavior/Behavior.hpp>

namespace TELEKYB_NAMESPACE {

BehaviorInterface::BehaviorInterface(Behavior& behavior_)
	: behavior(behavior_), nodeHandle(behavior.getNodeHandle())
{
	setNextBehavior = nodeHandle.advertiseService(
			BEHAVIOR_SETNEXTBEHAVIOR, &BehaviorInterface::setNextBehaviorCB , this);
	getNextBehavior = nodeHandle.advertiseService(
			BEHAVIOR_GETNEXTBEHAVIOR, &BehaviorInterface::getNextBehaviorCB , this);
	setParameterInitialized = nodeHandle.advertiseService(
			BEHAVIOR_SETPARAMINIT, &BehaviorInterface::setParameterInitializedCB , this);

}

BehaviorInterface::~BehaviorInterface()
{

}


// ServiceServers Callbacks
bool BehaviorInterface::setNextBehaviorCB(
		telekyb_srvs::BehaviorInput::Request& request,
		telekyb_srvs::BehaviorInput::Response& response)
{
	return behavior.setNextBehavior( Behavior::behaviorFromID(request.behaviorID) );
}

bool BehaviorInterface::getNextBehaviorCB(
		telekyb_srvs::BehaviorOutput::Request& request,
		telekyb_srvs::BehaviorOutput::Response& response)
{
	// nextBehavior is always valid!
	Behavior* b = behavior.getNextBehavior();
	response.behaviorID = b->getID();
	response.behaviorName = b->getName();
	return true;
}

bool BehaviorInterface::setParameterInitializedCB(
		telekyb_srvs::BoolInput::Request& request,
		telekyb_srvs::BoolInput::Response& response)
{
	behavior.setParameterInitialized((bool)request.input);
	return true;
}

}
