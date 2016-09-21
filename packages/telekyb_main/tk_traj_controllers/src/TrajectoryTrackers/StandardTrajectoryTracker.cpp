/*
 * StandardTrajectoryTracker.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include "StandardTrajectoryTracker.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKCommands.h>

//
#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_defines/physic_defines.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( trajectory_trackers_plugin::StandardTrajectoryTracker, TELEKYB_NAMESPACE::TrajectoryTracker);

namespace trajectory_trackers_plugin {


// Options
StandardTrajectoryTrackerOptions::StandardTrajectoryTrackerOptions()
	: OptionContainer("StandardTrajectoryTracker")
{
	tCommandsTopic = addOption<std::string>("tCommandsTopic","Topic for publishing telekyb_msgs::TKCommands",
			"commands", false, true);
	tPluginLookupName = addOption<std::string>("tPluginLookupName",
			"Specifies the Mass Estimation Plugin for the " + getOptionContainerNamespace(),
			"parameter_estimators_plugin::StandardMassEstimator", false, true);

}


StandardTrajectoryTracker::StandardTrajectoryTracker()
	: tDoMassEstimation( NULL ),
	  meLoader( "tk_param_estimator", "tk_param_estimator::MassEstimator" ),
// 	  massEstimator( NULL ),
	  nodeHandle( TELEKYB_NAMESPACE::ROSModule::Instance().getMainNodeHandle() ),
	  commandNodeHandle( nodeHandle, TELEKYB_COMMAND_NODESUFFIX )
{

}

StandardTrajectoryTracker::~StandardTrajectoryTracker()
{
	if (massEstimator) {
		massEstimator->destroy();
// 		delete massEstimator;
	}
}

void StandardTrajectoryTracker::initialize()
{
	// Important first create Publisher, before receiving CallBacks
	tTcCommandsPub = commandNodeHandle.advertise<telekyb_msgs::TKCommands>(options.tCommandsTopic->getValue(),1);

	// CurrentState
	currentInput.setAcceleration( TELEKYB_NAMESPACE::Acceleration3D(0.0, 0.0, GRAVITY) );
	currentInput.setYawRate(0.0);

	//std::string tkStateTopicName = StateEstimatorController::Instance().getSePublisherTopic();
	//tTcStateSub = nodeHandle.subscribe(tkStateTopicName,1,&StandardTrajectoryTracker::tkStateCB, this);
	positionControl = new TELEKYB_NAMESPACE::PositionControl;
	yawControl = new TELEKYB_NAMESPACE::YawControl();

	try {
		massEstimator = meLoader.createInstance(options.tPluginLookupName->getValue());
		// Currently RunTime Switch is not supported. This has to be changed then.
		massEstimator->initialize();

	} catch (pluginlib::PluginlibException& e) {
		ROS_FATAL("Trajectory Tracker %s failed to load: %s", options.tPluginLookupName->getValue().c_str(), e.what());
		//ROS_BREAK();
		ros::shutdown();
	}

	// Get Option
	tDoMassEstimation = TELEKYB_NAMESPACE::OptionContainer::getGlobalOptionByName<bool>("TrajectoryController","tDoMassEstimation");
	if (!tDoMassEstimation) {
		ROS_ERROR("Unable to get Option TrajectoryController/tDoMassEstimation. Quitting...");
		ros::shutdown();
	}

	// fill currentMass with Initial Value!
	currentMass = massEstimator->getInitialMass();

	// sanity check
//	TrajectoryTracker *tracker = new StandardTrajectoryTracker();
//	delete tracker;
}
//	virtual void willBecomeActive() = 0;
//	virtual void willBecomeInActive() = 0;
void StandardTrajectoryTracker::destroy()
{

}

std::string StandardTrajectoryTracker::getName() const
{
	return "StandardTrajectoryTracker";
}


void StandardTrajectoryTracker::trajectoryCB(const TELEKYB_NAMESPACE::TKTrajectory& trajectory)
{
	boost::mutex::scoped_lock currentInputLock(currentInputMutex);
	currentInput = trajectory;
}

void StandardTrajectoryTracker::stateCB(const TELEKYB_NAMESPACE::TKState& state)
{
	// new State Message. Triggers control step!
	//ROS_INFO("Received new TKState!");
	TELEKYB_NAMESPACE::Vector3D rpyOrientation = state.getEulerRPY();

	// Position Control
	TELEKYB_NAMESPACE::PosCtrlOutput pcOutput;
	// lock
	boost::mutex::scoped_lock currentInputLock(currentInputMutex);
  
	positionControl->run(currentInput, state, currentMass, pcOutput);
	// Yaw Control
	TELEKYB_NAMESPACE::YawCtrlOutput ycOutput;
	yawControl->run(currentInput, state, ycOutput);
	
	// unlock
	currentInputLock.unlock();



	//ROS_INFO("Roll: %f, Pitch: %f, Thrust %f", pcOutput.comRoll, pcOutput.comPitch, pcOutput.comThrust);

	// Mass Estimation only when enabled.
	if (tDoMassEstimation->getValue()) {
		MassEstimInput meInput;
		meInput.roll = rpyOrientation(0);
		meInput.pitch = rpyOrientation(1);
		meInput.thrust = pcOutput.comThrust;
		meInput.vertVel = state.linVelocity(2); // z

		MassEstimOutput meOutput;
		massEstimator->run(meInput, meOutput);
		// update Mass.
		currentMass = meOutput.estMass;
	}


	//positionControl.setMass(meOutput.estMass);


	telekyb_msgs::TKCommands cmdMsg;

	cmdMsg.header.stamp = ros::Time::now();
	cmdMsg.roll = pcOutput.comRoll;
	cmdMsg.pitch = pcOutput.comPitch;
	cmdMsg.yaw = ycOutput.comYaw;
	cmdMsg.thrust = pcOutput.comThrust;
	// mass
	cmdMsg.mass = currentMass;


	tTcCommandsPub.publish(cmdMsg);

}


}
