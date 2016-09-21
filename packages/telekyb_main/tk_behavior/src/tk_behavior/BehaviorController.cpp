/*
 * BehaviorController.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include <tk_behavior/BehaviorController.hpp>
#include <telekyb_base/ROS.hpp>

#include <telekyb_defines/physic_defines.hpp>

// VERY TEMPORARY!
#include <tk_trajprocessor/TrajectoryProcessorController.hpp>

namespace TELEKYB_NAMESPACE {

BehaviorController::BehaviorController()
	: behaviorNodeHandle( ROSModule::Instance().getMainNodeHandle(), TELEKYB_BEHAVIOR_NODESUFFIX ),
	  activeBehavior( NULL ),
	  trajProcCtrl( TrajectoryProcessorController::Instance() ) // TEMPORARY!!!
{
	bcInterface = new BehaviorControllerInterface(*this);


	// Default lastInput for Ground.
	lastInput.setAcceleration( Acceleration3D(0.0, 0.0, GRAVITY) );
	lastInput.setYawRate(0.0);
}

//** This is like a Constructor. It's called by the Singleton creator DIRECTLY AFTER the actual constructor. **/
void BehaviorController::initialize()
{
	systemBehaviorContainer = new SystemBehaviorContainer();
	// Start in Ground Behavior
	activeBehavior = new ActiveBehaviorContainer( systemBehaviorContainer->getGround() );
	
	std::string tkStateTopicName;
	if (options.tStateEstimationTopic->isOnInitialValue()){
		tkStateTopicName = StateEstimatorController::Instance().getSePublisherTopic();
	} else {
	// tkStateTopicName is an absolute PATH!
		tkStateTopicName = options.tStateEstimationTopic->getValue();
	}
	tStateSub = behaviorNodeHandle.subscribe(tkStateTopicName,1,&BehaviorController::tkStateCB, this);
}

void BehaviorController::tkStateCB(const telekyb_msgs::TKState::ConstPtr& tkStateMsg)
{
	TKState currentState(*tkStateMsg);

	// This is where the magic happens
	activeBehavior->trajectoryStep(currentState, lastInput);

	// send to ROS
	bcInterface->publishTKTrajectory(lastInput);
	// send to receiver.
	trajProcCtrl.trajInputStep(lastInput);
}

BehaviorController::~BehaviorController()
{
	delete bcInterface;
	delete activeBehavior;
	delete systemBehaviorContainer;
}


const ros::NodeHandle& BehaviorController::getBehaviorNodeHandle() const
{
	return behaviorNodeHandle;
}

Behavior* BehaviorController::getActiveBehavior() const
{
	return activeBehavior->getActive();
}

bool BehaviorController::switchToBehavior(Behavior* newBehavior)
{
	return activeBehavior->switchToBehavior(newBehavior);
}

const SystemBehaviorContainer& BehaviorController::getSystemBehaviorContainer() const
{
	return *systemBehaviorContainer;
}

bool BehaviorController::switchToNormalBrake()
{
	return activeBehavior->switchToBehavior(systemBehaviorContainer->getNormalBrake());
}

bool BehaviorController::switchToEmergencyLand()
{
	return activeBehavior->switchToBehavior(systemBehaviorContainer->getEmergencyLand());
}

void BehaviorController::activeBehaviorChanged()
{
	Behavior* newActive = activeBehavior->getActive();

	// Turn TrajProcCtrl if not in Air
	if (trajProcCtrl.isActive() && newActive->getType() != BehaviorType::Air) {
		trajProcCtrl.setInActive();
	}
	if (!trajProcCtrl.isActive() && newActive->getType() == BehaviorType::Air) {
		trajProcCtrl.setActive(this);
	}

	// tell the world about it
	bcInterface->publishActiveBehavior();
}



//---------------
// Singleton Stuff
BehaviorController* BehaviorController::instance = NULL;

BehaviorController& BehaviorController::Instance() {
	if (!instance) {
		instance = new BehaviorController();
		instance->initialize();
	}
	return *instance;
}

const BehaviorController* BehaviorController::InstancePtr() {
	if (!instance) {
		instance = new BehaviorController();
		instance->initialize();
	}

	return instance;
}

bool BehaviorController::HasInstance()
{
	return (instance != NULL);
}

void BehaviorController::ShutDownInstance() {
	if (instance) {
		delete instance;
	}

	instance = NULL;
}


}
