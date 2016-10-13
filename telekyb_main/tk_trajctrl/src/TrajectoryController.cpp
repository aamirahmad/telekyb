/*
 * TrajectoryController.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include <tk_trajctrl/TrajectoryController.hpp>

#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKCommands.h>

//
#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_defines/physic_defines.hpp>

namespace TELEKYB_NAMESPACE {

//TrajectoryController::TrajectoryController()
//	: massEstimation( options.tInitialMass->getValue() ),
//	  currentMass( options.tInitialMass->getValue() ),
//	  nodeHandle( ROSModule::Instance().getMainNodeHandle() ),
//	  commandNodeHandle( nodeHandle, TELEKYB_COMMAND_NODESUFFIX )
//{
//	// Important first create Publisher, before receiving CallBacks
//	tTcCommandsPub = commandNodeHandle.advertise<telekyb_msgs::TKCommands>(options.tCommandsTopic->getValue(),1);
//
//	// CurrentState
//	currentInput.setAcceleration( Acceleration3D(0.0, 0.0, GRAVITY) );
//	currentInput.setYawRate(0.0);
//
//	std::string tkStateTopicName = StateEstimatorController::Instance().getSePublisherTopic();
//	tTcStateSub = nodeHandle.subscribe(tkStateTopicName,1,&TrajectoryController::tkStateCB, this);
//}

TrajectoryController::TrajectoryController()
	: ttLoader( "tk_trajctrl", std::string( TELEKYB_NAMESPACE_STRING ) + "::TrajectoryTracker" ),
// 	  activeTrajectoryTracker( NULL ),
	  nodeHandle( ROSModule::Instance().getMainNodeHandle() )
{

}

TrajectoryController::~TrajectoryController()
{
	if (activeTrajectoryTracker) {
		activeTrajectoryTracker->destroy();
// 		delete activeTrajectoryTracker;
	}
}

void TrajectoryController::initialize()
{
	try {
		activeTrajectoryTracker = ttLoader.createInstance(options.tPluginLookupName->getValue());
		// Currently RunTime Switch is not supported. This has to be changed then.
		activeTrajectoryTracker->initialize();


		// only subscribe after successful load
		std::string tkStateTopicName;
		if (options.tStateEstimationTopic->isOnInitialValue()){
			tkStateTopicName = StateEstimatorController::Instance().getSePublisherTopic();
		} else {
			tkStateTopicName = options.tStateEstimationTopic->getValue();
		}

		tTcStateSub = nodeHandle.subscribe(tkStateTopicName,1,&TrajectoryController::tkStateCB, this);

	} catch (pluginlib::PluginlibException& e) {
		ROS_FATAL("Trajectory Tracker %s failed to load: %s", options.tPluginLookupName->getValue().c_str(), e.what());
		//ROS_BREAK();
		ros::shutdown();
	}
}


void TrajectoryController::trajInputStep(const TKTrajectory& nextInput)
{
	activeTrajectoryTracker->trajectoryCB(nextInput);
}

//void TrajectoryController::toggleMassEstimation(bool toggleME)
//{
//	if (!options.tCompletelyDisableME->getValue()) {
//		options.tDoMassEstimation->setValue(toggleME);
//	}
//}

void TrajectoryController::tkStateCB(const telekyb_msgs::TKState::ConstPtr& msg)
{
	// new State Message. Triggers control step!
	//ROS_INFO("Received new TKState!");
	TKState state(*msg);
	activeTrajectoryTracker->stateCB(state);
}


//---------------
// Singleton Stuff
TrajectoryController* TrajectoryController::instance = NULL;

TrajectoryController& TrajectoryController::Instance() {
	if (!instance) {
		instance = new TrajectoryController();
		instance->initialize();
	}
	return *instance;
}

TrajectoryController* TrajectoryController::InstancePtr() {
	if (!instance) {
		instance = new TrajectoryController();
		instance->initialize();
	}

	return instance;
}

bool TrajectoryController::HasInstance()
{
	return (instance != NULL);
}

void TrajectoryController::ShutDownInstance() {
	if (instance) {
		delete instance;
	}

	instance = NULL;
}

}
