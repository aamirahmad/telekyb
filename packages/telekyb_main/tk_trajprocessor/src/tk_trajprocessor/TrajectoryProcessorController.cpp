/*
 * TrajectoryProcessorController.cpp
 *
 *  Created on: Dec 13, 2011
 *      Author: mriedel
 */

#include <tk_trajprocessor/TrajectoryProcessorController.hpp>

#include <telekyb_base/ROS.hpp>

#include <tk_trajctrl/TrajectoryController.hpp>

namespace TELEKYB_NAMESPACE {

TrajectoryProcessorController::TrajectoryProcessorController()
	: trajProcessorNodeHandle( ROSModule::Instance().getMainNodeHandle(), TELEKYB_TRAJPROC_NODESUFFIX ),
	  seController( StateEstimatorController::Instance() ),
	  trajController( TrajectoryController::Instance() ),
	  active(false),
	  switcher(NULL)
{


}

TrajectoryProcessorController::~TrajectoryProcessorController()
{

}

void TrajectoryProcessorController::initialize()
{
	// load Trajectory Modules
	std::vector< std::string > trajectoryModulesStrings = options.tTrajectoryModules->getValue();
	for (unsigned int i = 0; i < trajectoryModulesStrings.size(); ++i) {
		trajModuleContainer.loadTrajectoryModule(trajectoryModulesStrings[i]);
	}

	// did you get at least one state?
	if (! seController.waitForFirstState( Time(options.tInitialStateTimeout->getValue()) ) ) {
		// did not receive State
		ROS_ERROR("TrajectoryProcessorController: Did not receive initial TKState within Timeout! Using uninitialized! This is dangerous!");
	}

	// this should block till state becomes available
	lastState = seController.getLastState();
	
	std::string tkStateTopicName;
	if (options.tStateEstimationTopic->isOnInitialValue()){
			tkStateTopicName = StateEstimatorController::Instance().getSePublisherTopic();
	} else {
		// tkStateTopicName is an absolute PATH!
		tkStateTopicName = options.tStateEstimationTopic->getValue();
	}
	tStateSub = trajProcessorNodeHandle.subscribe(tkStateTopicName,1,&TrajectoryProcessorController::tkStateCB, this);
}

void TrajectoryProcessorController::setActive(BehaviorSwitcher* switcher_)
{
	trajModuleContainer.activateAllTrajectoryModules();

	switcher = switcher_;
	active = true;
}

void TrajectoryProcessorController::setInActive()
{
	trajModuleContainer.deactivateAllTrajectoryModules();
	active = false;
	switcher = NULL;
}

bool TrajectoryProcessorController::isActive() const
{
	return active;
}

BehaviorSwitcher* TrajectoryProcessorController::getBehaviorSwitcher() const
{
	return switcher;
}

void TrajectoryProcessorController::trajInputStep(const TKTrajectory& input)
{
	if (active) {
		TKTrajectory trajToProcess = input;

		// call accordingly:
		switch (trajToProcess.getGlobalPositionControlType().index()) {
			case GlobalPosControlType::Position:
				//ROS_INFO("Processing Position Mode");
				trajModuleContainer.trajectoryStepPosition(lastState, trajToProcess);
				break;
			case GlobalPosControlType::Velocity:
				//ROS_INFO("Processing Velocity Mode");
				trajModuleContainer.trajectoryStepVelocity(lastState, trajToProcess);
				break;
			case GlobalPosControlType::Acceleration:
				//ROS_INFO("Processing Acceleration Mode");
				trajModuleContainer.trajectoryStepAcceleration(lastState, trajToProcess);
				break;
			case GlobalPosControlType::Mixed:
				//ROS_INFO("Processing Mixed Mode");
				trajModuleContainer.trajectoryStepOther(lastState, trajToProcess);
				break;
			default:
				break;
		}

		// send to TrajController
		trajController.trajInputStep(trajToProcess);
	} else {
		// inactive just pass on
		trajController.trajInputStep(input);
	}
}

void TrajectoryProcessorController::tkStateCB(const telekyb_msgs::TKState::ConstPtr& tkStateMsg)
{
	boost::mutex::scoped_lock lastStateLock(lastStateMutex);
	lastState = *tkStateMsg;
}



//---------------
// Singleton Stuff
TrajectoryProcessorController* TrajectoryProcessorController::instance = NULL;

TrajectoryProcessorController& TrajectoryProcessorController::Instance() {
	if (!instance) {
		instance = new TrajectoryProcessorController();
		instance->initialize();
	}
	return *instance;
}

const TrajectoryProcessorController* TrajectoryProcessorController::InstancePtr() {
	if (!instance) {
		instance = new TrajectoryProcessorController();
		instance->initialize();
	}

	return instance;
}

bool TrajectoryProcessorController::HasInstance()
{
	return (instance != NULL);
}

void TrajectoryProcessorController::ShutDownInstance() {
	if (instance) {
		delete instance;
	}

	instance = NULL;
}


} /* namespace telekyb */
