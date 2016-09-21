/*
 * Calibrator.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#include "Calibrator.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>

// Options
CalibratorOptions::CalibratorOptions()
	: OptionContainer("CalibratorOptions")
{
	robotID = addOption<int>("robotID", "Specify the robotID of the TeleKybCore to connect to.", 0, false, true);
	tJoystickTopic = addOption<std::string>("tJoystickTopic",
			"Joysticktopic to use (sensor_msgs::Joy)", "/TeleKyb/tJoy/joy", false, true);
	tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!", false, false, true);
}

Calibrator::Calibrator()
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

	setupCalibrator();
}

Calibrator::~Calibrator()
{
	if (mkInterface) { delete mkInterface; }

	delete core;
}

void Calibrator::setupCalibrator()
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
	takeOff.getOptionContainer().getOption("tTakeOffDestination").set(Position3D(0.2,-2.0,-1.0));
	takeOff.getOptionContainer().getOption("tTakeOffVertically").set(false);

	// done
	takeOff.setParameterInitialized(true);

	land.getOptionContainer().getOption("tLandDestination").set(Position3D(0.0,0.0,0.0));
	land.setParameterInitialized(true);


	calibrator = bController->loadBehavior("telekyb_behavior::Calibrator");

	if ( calibrator.isNull() ) {
		ROS_FATAL("Unable to load Calibrator!!!");
		//ROS_BREAK();
		ros::shutdown();
	}

	// configure calibrator
	calibrator.setNextBehavior(land);
	calibrator.setParameterInitialized(true);


	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV not in Ground Behavior during Startup");
		ros::shutdown();
	}

	// lastly start Controller
	joySub = mainNodeHandle.subscribe(options.tJoystickTopic->getValue()
			, 10, &Calibrator::joystickCB, this);
}




void Calibrator::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
	// use button 2
	if (msg->buttons.size() < 9) {
		ROS_ERROR("Jostick does not publish enough buttons.");
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
			MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
			if (!mkInterface->updateMKValue(motorState)) {
				ROS_ERROR("Could not get Motorstate!");
				return;
			}
			if (motorState.value == MotorState::On) {
				bController->switchBehavior(takeOff);
			} else {
				ROS_ERROR("Motors have to be on for liftOff!");
				return;
			}
			// takeoff

		} else {
			// flying -> land
			bController->switchBehavior(land);
		}
	}

	if (msg->buttons[3]) {
		bController->switchBehavior(calibrator);
	}


	// what to do

}

void Calibrator::activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior)
{
	if (newActiveBehavior == hover) {
		// sleep 5s and land
		Time sleepTime(5);
		sleepTime.sleep();
		bController->switchBehavior(calibrator);
	}

//	if (newActiveBehavior == calibrator) {
//		oController->getOption("tIntegGain","PositionControl").get(saveIntegralGain);
//		oController->getOption("tIntegGain","PositionControl").set(0.0);
//	}
//
//	if (newActiveBehavior == land) {
//		oController->getOption("tIntegGain","PositionControl").set(saveIntegralGain);
//	}

	if (mkInterface && newActiveBehavior == ground) {
		MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
		motorState.value = MotorState::Off;
		mkInterface->setMKValue(motorState);
	}

}

