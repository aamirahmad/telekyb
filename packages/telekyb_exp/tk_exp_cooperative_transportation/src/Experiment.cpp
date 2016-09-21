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
	tExternalTrajectoryTopic = addOption<std::string>("tExternalTrajectoryTopic", "topic of the ext trajectory", "ExtTraj", false, true);
	tTakeOffDestination = addOption<Position3D>("tTakeOffDestination", "destination of the takoff", Position3D(0.0,0.0,-1.0), false, false);
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
	takeOff.getOptionContainer().getOption("tTakeOffDestination").set(options.tTakeOffDestination->getValue());
	takeOff.getOptionContainer().getOption("tTakeOffVertically").set<bool>(false);
	takeOff.getOptionContainer().getOption("tTakeOffInPosition").set<bool>(true);
	takeOff.getOptionContainer().getOption("tTakeOffVelocity").set<double>(0.3);
	takeOff.getOptionContainer().getOption("tTakeOffDestinationRadius").set<double>(0.08);

	// done
	takeOff.setParameterInitialized(true);


	land.getOptionContainer().getOption("tLandVelocity").set<double>(0.4);
	land.getOptionContainer().getOption("tLandDestination").set(Position3D(0.0,0.0,0.0));
	land.getOptionContainer().getOption("tLandVertically").set<bool>(true);
	land.getOptionContainer().getOption("tLandInPosition").set<bool>(true);
	//	land.getOptionContainer().getOption("tLandDestinationHeight").set<double>(-0.25);
	land.setParameterInitialized(true);


	trajForceInput = bController->loadBehavior("tk_be_common/TrajectoryForceInput");

	if ( trajForceInput.isNull() ) {
		ROS_FATAL("Unable to load trajForceInput!!!");
		//ROS_BREAK();
		ros::shutdown();
	}

	// setup joystick
// 	joystick.getOptionContainer().getOption("tJoystickTopic").set(options.tJoystickTopic->getValue());
	trajForceInput.getOptionContainer().getOption("tTrajectoryTopicName").set<std::string>(options.tExternalTrajectoryTopic->getValue());
	trajForceInput.setParameterInitialized(true);

	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV not in Ground Behavior during Startup");
		ros::shutdown();
	}

	// lastly start Controller
	joySub = mainNodeHandle.subscribe(options.tJoystickTopic->getValue()
			, 10, &Experiment::joystickCB, this);
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
		bController->switchBehavior(trajForceInput);
	}

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

