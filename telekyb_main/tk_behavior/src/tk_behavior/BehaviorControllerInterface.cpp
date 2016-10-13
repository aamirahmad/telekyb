/*
 * BehaviorInterface.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: mriedel
 */

#include <tk_behavior/BehaviorControllerInterface.hpp>

#include <tk_behavior/BehaviorController.hpp>

#include <telekyb_msgs/Behavior.h>
#include <telekyb_msgs/TKTrajectory.h>

namespace TELEKYB_NAMESPACE {

BehaviorControllerInterface::BehaviorControllerInterface(BehaviorController& behaviorController_)
	: behaviorController( behaviorController_ ), nodeHandle(behaviorController.getBehaviorNodeHandle())
{
	// TODO Auto-generated constructor stub
	getAvailableBehaviorsClient = nodeHandle.advertiseService(
			BEHAVIOR_GETAVAILABLEBEHAVIORS, &BehaviorControllerInterface::getAvailableBehaviorsCB , this);
	loadBehavior = nodeHandle.advertiseService(
			BEHAVIOR_LOADBEHAVIOR, &BehaviorControllerInterface::loadBehaviorCB , this);
	unloadBehavior = nodeHandle.advertiseService(
			BEHAVIOR_UNLOADBEHAVIOR, &BehaviorControllerInterface::unloadBehaviorCB , this);
	switchBehavior = nodeHandle.advertiseService(
			BEHAVIOR_SWITCHBEHAVIOR, &BehaviorControllerInterface::switchBehaviorCB , this);
	getSystemBehavior = nodeHandle.advertiseService(
			BEHAVIOR_GETSYSTEMBEHAVIOR, &BehaviorControllerInterface::getSystemBehaviorCB , this);
	getActiveBehavior = nodeHandle.advertiseService(
			BEHAVIOR_GETACTIVEBEHAVIOR, &BehaviorControllerInterface::getActiveBehaviorCB , this);
	emergencyLand = nodeHandle.advertiseService(
			BEHAVIOR_EMERGENCYLAND, &BehaviorControllerInterface::emergencyLandCB , this);
	normalBrake = nodeHandle.advertiseService(
			BEHAVIOR_NORMALBRAKE, &BehaviorControllerInterface::normalBrakeCB , this);

	activeBehaviorPub = nodeHandle.advertise<telekyb_msgs::Behavior>(BEHAVIOR_BEHAVIORCHANGETOPIC, 1);
	trajectoryPub = nodeHandle.advertise<telekyb_msgs::TKTrajectory>(BEHAVIOR_BEHAVIORTRAJECTORY, 1);
}

BehaviorControllerInterface::~BehaviorControllerInterface() {
	// TODO Auto-generated destructor stub
}

bool BehaviorControllerInterface::getAvailableBehaviorsCB(
		telekyb_srvs::StringArrayOutput::Request& request,
		telekyb_srvs::StringArrayOutput::Response& response)
{
	behaviorContainer.getAvailableBehaviors(response.output);
	return true;
}

bool BehaviorControllerInterface::loadBehaviorCB(
		telekyb_srvs::BehaviorInputOutput::Request& request,
		telekyb_srvs::BehaviorInputOutput::Response& response)
{
	const Behavior *b = behaviorContainer.loadBehavior(request.behaviorName);
	response.behaviorID = (uint64_t)b;
	if ( b ) { response.behaviorName = b->getName(); } // this should be identical to input
	return b != NULL;
}

bool BehaviorControllerInterface::unloadBehaviorCB(
		telekyb_srvs::BehaviorInput::Request& request,
		telekyb_srvs::BehaviorInput::Response& response)
{
	return behaviorContainer.unloadBehavior(Behavior::behaviorFromID(request.behaviorID));
}

bool BehaviorControllerInterface::switchBehaviorCB(
		telekyb_srvs::BehaviorInput::Request& request,
		telekyb_srvs::BehaviorInput::Response& response)
{
	return behaviorController.switchToBehavior(Behavior::behaviorFromID(request.behaviorID));
}

bool BehaviorControllerInterface::getSystemBehaviorCB(
		telekyb_srvs::BehaviorInputOutput::Request& request,
		telekyb_srvs::BehaviorInputOutput::Response& response)
{
	Behavior* b = behaviorController.getSystemBehaviorContainer().getBehaviorByName(request.behaviorName);
	// be can be NULL
	response.behaviorID = Behavior::behaviorToID(b);
	if ( b ) { response.behaviorName = b->getName(); }  // this should be identical to input
	return b != NULL;
}

bool BehaviorControllerInterface::getActiveBehaviorCB(
		telekyb_srvs::BehaviorOutput::Request& request,
		telekyb_srvs::BehaviorOutput::Response& response)
{
	// b IS ALWAYS VALID!
	Behavior* b = behaviorController.getActiveBehavior();
	// b cannot be NULL
	response.behaviorID = b->getID();
	response.behaviorName = b->getName();
	return true;
}

bool BehaviorControllerInterface::emergencyLandCB(
		std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response)
{
	return behaviorController.switchToEmergencyLand();
}

bool BehaviorControllerInterface::normalBrakeCB(
		std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response)
{
	return behaviorController.switchToNormalBrake();
}

void BehaviorControllerInterface::publishActiveBehavior()
{
	// b IS ALWAYS VALID!
	Behavior* b = behaviorController.getActiveBehavior();
	telekyb_msgs::Behavior msg;
	msg.header.stamp = ros::Time::now();
	msg.behaviorID = b->getID();
	msg.behaviorName = b->getName();

	activeBehaviorPub.publish(msg);
}

void BehaviorControllerInterface::publishTKTrajectory(const TKTrajectory& trajectory)
{
	telekyb_msgs::TKTrajectory msg;
    // convert the TELEKYB_NAMESPACE::TKTrajectory to a telekyb_msgs::TKTrajectory
	trajectory.toTKTrajMsg(msg);
    // publish the given trajectory
	trajectoryPub.publish(msg);
}

}
