/*
 * Experiment.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#include "Experiment.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>

// Options
ExperimentOptions::ExperimentOptions()
	: OptionContainer("ExperimentOptions")
{
	robotID = addOption<int>("robotID", "Specify the robotID of the TeleKybCore to connect to.", 0, false, true);
	tJoystickTopic = addOption<std::string>("tJoystickTopic",
			"Joysticktopic to use (sensor_msgs::Joy)", "/TeleKyb/tJoy/joy", false, true);
	tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!", false, false, true);
	tTrajPlaybackFilename = addOption<std::string>("tTrajPlaybackFilename",
			"Name of the file containing te trajectory to follow", "Traj.txt", false, false);

	tUseJoystick  = addOption<bool>("tUseJoystick",
			"If true, uses joystick to start motors, takeoff, start trajectory and land, if false, uses time.", true, false, false);
	tToggleMotorsTime  = addOption<double>("tToggleMotorsTime",
			"Time in which motors are started if tUseJoystick is false", 5.0, false, false);
	tTakeoffTime  = addOption<double>("tTakeoffTime",
			"Time in which takoff starts if tUseJoystick is false", 10.0, false, false);
	tTrajectoryStarts  = addOption<double>("tTrajectoryStarts",
			"Time in which trajectory playback starts if tUseJoystick is false", 20.0, false, false);
}

Experiment::Experiment()
	: mainNodeHandle( ROSModule::Instance().getMainNodeHandle() ), core(NULL), mkInterface(NULL)
{
	core = telekyb_interface::TeleKybCore::getTeleKybCore(options.robotID->getValue());
	if (!core) {
		// fail
		ros::shutdown();
		return;
	}

	//options.tUseMKInterface->setValue(true);
	if (options.tUseMKInterface->getValue()) {
		ROS_INFO("Creating MKInterface!");
		// use MKInterface
		mkInterface = telekyb_interface::MKInterface::getMKInterface(options.robotID->getValue());
		//mkInterface = telekyb_interface::MKInterface::getMKInterface(1); // BEWARE TEMPORARY!!!
		if (!mkInterface) {
			// fail
			ros::shutdown();
			return;
		}
	}

	bController = core->getBehaviorController();
	oController = core->getOptionController();

	//activeBehavior = bController->getActiveBehaviorReference();
	bController->setActiveBehaviorListener(this);

	activeBehaviorPtr = bController->getActiveBehaviorPointer();

	setupExperiment();
}

Experiment::~Experiment()
{
	if (mkInterface) { delete mkInterface; }

	delete core;
}

void Experiment::setupExperiment()
{
	// load Behaviors
	ground = bController->getSystemBehavior("tk_behavior/Ground");
	hover = bController->getSystemBehavior("tk_behavior/Hover");
	normalBreak = bController->getSystemBehavior("tk_behavior/NormalBrake");
	takeOff = bController->getSystemBehavior("tk_behavior/TakeOff");
	land = bController->getSystemBehavior("tk_behavior/Land");


	// sanity check
	if (ground.isNull() || hover.isNull() || normalBreak.isNull() || takeOff.isNull() || land.isNull()) {
		ROS_FATAL("Unable to get SystemBehavior!!!");
		//ROS_BREAK();
		ros::shutdown();
	}


	// setup takeoff
	takeOff.getOptionContainer().getOption("tTakeOffVelocity").set<double>(0.3);
	takeOff.getOptionContainer().getOption("tTakeOffInPosition").set<bool>(true);
	takeOff.getOptionContainer().getOption("tTakeOffDestination").set(Position3D(0.0,0.0,-1.0));
	takeOff.getOptionContainer().getOption("tTakeOffVertically").set(true);

	// done
	takeOff.setParameterInitialized(true);


	land.getOptionContainer().getOption("tLandVelocity").set<double>(0.4);
	land.getOptionContainer().getOption("tLandDestination").set(Position3D(0.0,0.0,0.0));
//	land.getOptionContainer().getOption("tLandDestinationHeight").set<double>(-0.25);
	land.setParameterInitialized(true);
	
	
	normalBreak.getOptionContainer().getOption("tBrakeInPosition").set<bool>(true);
	normalBreak.setParameterInitialized(true);
	
	
	
	trajPlayback = bController->loadBehavior("tk_be_common/TrajPlayback");
	if ( trajPlayback.isNull() ) {
		ROS_FATAL("Unable to load Trajectory Playback!!!");
		//ROS_BREAK();
		ros::shutdown();
	}
	trajPlayback.getOptionContainer().getOption("tTrajectoryFilename").set(options.tTrajPlaybackFilename->getValue());
	trajPlayback.setParameterInitialized(true);
	
	
	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV not in Ground Behavior during Startup");
		ros::shutdown();
	}
	
	// lastly start Controller in joystick CB if using joystick, using time otherwise
	if (options.tUseJoystick->getValue()){
		joySub = mainNodeHandle.subscribe(options.tJoystickTopic->getValue(), 10, &Experiment::joystickCB, this);
	}
	else{
		timeSub = mainNodeHandle.createTimer(ros::Duration(0.05), &Experiment::timeCB, this);
	}

}


void Experiment::timeCB(const ros::TimerEvent&)
{
// 	// Emergency
// 	if (msg->buttons[6]) {
// 		ROS_WARN("Emergency Button pressed!");
// 		mkInterface->setEmergency();
// 	}

	double now = ros::Time::now().toSec();
	double timeToToggleMot = options.tToggleMotorsTime->getValue() - now;
	double timeToTakeoff = options.tTakeoffTime->getValue() - now;
	double timeToTrajectoryStart = options.tTrajectoryStarts->getValue() - now;
	
	if (mkInterface && *activeBehaviorPtr == ground &&  timeToToggleMot>-0.025 && timeToToggleMot<0.025) {
		ROS_INFO("Toggle Motorstate!");
		MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
		if (!mkInterface->updateMKValue(motorState)) {
			ROS_ERROR("Could not get Motorstate!");
			return;
		}
		if (motorState.value == MotorState::On) {
			// stop
			motorState.value = MotorState::Off;
			mkInterface->setMKValue(motorState);
		} else if (motorState.value == MotorState::Off) {
			// start
			motorState.value = MotorState::Init;
			mkInterface->setMKValue(motorState);
		} else if (motorState.value == MotorState::Init) {
			motorState.value = MotorState::On;
			mkInterface->setMKValue(motorState);
		}
	}

	if (timeToTakeoff>-0.025 && timeToTakeoff<0.025 ) {
		if (*activeBehaviorPtr == ground) {

			if (mkInterface) {
				MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
				if (!mkInterface->updateMKValue(motorState)) {
					ROS_ERROR("Could not get Motorstate for liftoff!");
					return;
				}
				if (motorState.value == MotorState::On) {
					bController->switchBehavior(takeOff);
					return;
				} else {
					ROS_ERROR("Motors have to be on for liftOff!");
					return;
				}
			}
			else
			{
				// takeoff
				bController->switchBehavior(takeOff);
			}
		}
// 			else {
// 			// flying -> land
// 			bController->switchBehavior(land);
// 		}
	}

// 	if (msg->buttons[3]) {
// 		bController->switchBehavior(joystick);
// 	}

	if (timeToTrajectoryStart>-0.025 && timeToTrajectoryStart<0.025 ) {
		bController->switchBehavior(trajPlayback);
	}
}

void Experiment::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
	// use button 2
	if (msg->buttons.size() < 9) {
		ROS_ERROR("Joystick does not publish enough buttons.");
		return;
	}

	// Emergency
	if (msg->buttons[6]) {
		ROS_WARN("Emergency Button pressed!");
		mkInterface->setEmergency();
	}

	// Button 1! toggle Motors for mkInterface Only in Ground
	if (mkInterface && *activeBehaviorPtr == ground && msg->buttons[0]) {
		ROS_INFO("Toggle Motorstate!");

		MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
		if (!mkInterface->updateMKValue(motorState)) {
			ROS_ERROR("Could not get Motorstate!");
			return;
		}

		if (motorState.value == MotorState::On) {
			// stop
			motorState.value = MotorState::Off;
			mkInterface->setMKValue(motorState);
		} else if (motorState.value == MotorState::Off) {
			// start
			motorState.value = MotorState::Init;
			mkInterface->setMKValue(motorState);
		} else if (motorState.value == MotorState::Init) {
			motorState.value = MotorState::On;
			mkInterface->setMKValue(motorState);
		}


	}

	if (msg->buttons[2]) {
		if (*activeBehaviorPtr == ground) {

			if (mkInterface) {
				MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
				if (!mkInterface->updateMKValue(motorState)) {
					ROS_ERROR("Could not get Motorstate for liftoff!");
					return;
				}
				if (motorState.value == MotorState::On) {
					bController->switchBehavior(takeOff);
					return;
				} else {
					ROS_ERROR("Motors have to be on for liftOff!");
					return;
				}
			}

			// takeoff
			bController->switchBehavior(takeOff);
		} else {
			// flying -> land
			bController->switchBehavior(land);
		}
	}

	if (msg->buttons[3]) {
// 		bController->switchBehavior(joystick);
	}

	if (msg->buttons[4]) {
		bController->switchBehavior(trajPlayback);
	}

	// right back
//	if (msg->buttons[5]) {
//		bController->switchBehavior(flyto1);
//	}

	// what to do
}

void Experiment::activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior)
{
// Automatically turn off Motors
	if (mkInterface && newActiveBehavior == ground) {
		MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
		motorState.value = MotorState::Off;
		mkInterface->setMKValue(motorState);
	}
}

