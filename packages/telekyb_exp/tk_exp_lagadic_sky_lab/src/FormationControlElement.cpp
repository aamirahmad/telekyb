/*
 * FormationControlElement.cpp
 *
 *  Created on: Feb 10, 2012
 *      Author: mriedel
 */

#include "FormationControlElement.hpp"

//#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>

FormationControlElement::FormationControlElement(int robotID_, bool useMKInterface_)
	: robotID(robotID_), core(NULL), mkInterface(NULL)
{
	core = telekyb_interface::TeleKybCore::getTeleKybCore(robotID);
	if (!core) {
		// fail
		ros::shutdown();
		return;
	}

	//options.tUseMKInterface->setValue(true);
	if (useMKInterface_) {
		ROS_INFO("Creating MKInterface!");
		// use MKInterface
		mkInterface = telekyb_interface::MKInterface::getMKInterface(robotID);
		if (!mkInterface) {
			// fail
			ROS_WARN("No MKInterface for Robot %d", robotID);
			//ros::shutdown();
			//return;
		}
	}

	bController = core->getBehaviorController();
	oController = core->getOptionController();

	//activeBehavior = bController->getActiveBehaviorReference();
	bController->setActiveBehaviorListener(this);
	activeBehaviorPtr = bController->getActiveBehaviorPointer();

	setupFormationControlElement();
}

FormationControlElement::~FormationControlElement()
{
	if (mkInterface) { delete mkInterface; }

	delete core;
}

void FormationControlElement::setupFormationControlElement()
{
	// load Behaviors
	ground = bController->getSystemBehavior("tk_behavior/Ground");
	hover = bController->getSystemBehavior("tk_behavior/Hover");
	normalBreak = bController->getSystemBehavior("tk_behavior/NormalBrake");
	takeOff = bController->getSystemBehavior("tk_behavior/TakeOff");
	land = bController->getSystemBehavior("tk_behavior/Land");


	// sanity check
	if (ground.isNull() || hover.isNull() || normalBreak.isNull() || takeOff.isNull() || land.isNull()) {
		ROS_FATAL("Unable to get SystemBehavior for UAV %d !!!", robotID);
		//ROS_BREAK();
		ros::shutdown();
	}


	// setup takeoff
	//takeOff.getOptionContainer().getOption("tTakeOffDestination").set(Position3D(0.0,0.0,-0.75));
	takeOff.getOptionContainer().getOption("tTakeOffVertically").set(true);
	if (!mkInterface) {
		takeOff.getOptionContainer().getOption("tTakeOffVelocity").set(0.4);
	} else {
		// Real QC
		takeOff.getOptionContainer().getOption("tTakeOffVelocity").set(0.4);
	}
	// done
	takeOff.setParameterInitialized(true);

	land.getOptionContainer().getOption("tLandDestination").set(Position3D(0.0,0.0,0.0));
	// how to do faster land velocity??
//	if (!mkInterface) {
//		takeOff.getOptionContainer().getOption("tLandVelocity").set(2.0);
//	}
	land.setParameterInitialized(true);


	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV %d not in Ground Behavior during Startup", robotID);
		ros::shutdown();
	}


	// I don't need any formation behavior
	/*
	formation = bController->loadBehavior("tk_formation/Formation");

	if ( formation.isNull() ) {
		ROS_FATAL("Unable to load FormationControl for UAV %d !!!", robotID);
		//ROS_BREAK();
		ros::shutdown();
	}

	// configure calibrator
	formation.setNextBehavior(land);
	//formation.setParameterInitialized(true);
	*/








	// Trajectory Input for Matlab
	trajectoryInput = bController->loadBehavior("tk_be_common/TrajectoryInput");

	if ( trajectoryInput.isNull() ) {
		ROS_FATAL("Unable to load TrajectoryInput for UAV %d !!!", robotID);
		//ROS_BREAK();
		ros::shutdown();
	}

	trajectoryInput.getOptionContainer().getOption("tTrajectoryTopicName").set<std::string>("InputTrajectory");
	trajectoryInput.setParameterInitialized(true);




	// Fly Back to original position
	flyBack = bController->loadBehavior("tk_be_common/TrajectoryInput");

	if ( flyBack.isNull() ) {
		ROS_FATAL("Unable to load Fly Back Controller for UAV %d !!!", robotID);
		//ROS_BREAK();
		ros::shutdown();
	}

	flyBack.getOptionContainer().getOption("tTrajectoryTopicName").set<std::string>("flyBack");
	flyBack.setParameterInitialized(true);

}

telekyb_interface::MKInterface* FormationControlElement::getMKInterfacePointer() const
{
	return mkInterface;
}

telekyb_interface::TeleKybCore* FormationControlElement::getTeleKybCorePointer() const
{
	return core;
}

bool FormationControlElement::mkSetEmergency()
{
	if (mkInterface) {
		return mkInterface->setEmergency();
	}
	return false;
}

bool FormationControlElement::mkToggleMotors()
{
	// Button 1! toggle Motors for mkInterface Only in Ground
	if (mkInterface && *activeBehaviorPtr == ground) {
		ROS_INFO("Toggle Motorstate!");

		MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
		if (!mkInterface->updateMKValue(motorState)) {
			ROS_ERROR("Could not get Motorstate!");
			return false;
		}

		if (motorState.value == MotorState::On) {
			// stop
			motorState.value = MotorState::Off;
			return mkInterface->setMKValue(motorState);
		} else if (motorState.value == MotorState::Off) {
			// start
			motorState.value = MotorState::Init;
			return mkInterface->setMKValue(motorState);
		} else if (motorState.value == MotorState::Init) {
			motorState.value = MotorState::On;
			return mkInterface->setMKValue(motorState);
		}
	}
	return false;
}

bool FormationControlElement::liftland()
{
	if (*activeBehaviorPtr == ground) {

		if (mkInterface) {
			MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
			if (!mkInterface->updateMKValue(motorState)) {
				ROS_ERROR("Could not get Motorstate!");
				return false;
			}
			if (motorState.value == MotorState::On) {
				bController->switchBehavior(takeOff);
			} else {
				ROS_ERROR("Motors have to be on for liftOff!");
				return false;
			}
		}
		// takeoff
		bController->switchBehavior(takeOff);
		return true;

	} else {
		// flying -> land
		bController->switchBehavior(land);
		return true;
	}

	return false;
}

void FormationControlElement::activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior)
{
//	if (newActiveBehavior == hover) {
//		// sleep 5s and land
//		Time sleepTime(0.1);
//		sleepTime.sleep();
//		//bController->switchBehavior(land);
//	}

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


telekyb_interface::Behavior FormationControlElement::getFormationBehavior() const
{
	return trajectoryInput;
}


void FormationControlElement::switchIntoFormation() const
{
	bController->switchBehavior(trajectoryInput);
}

void FormationControlElement::switchIntoNormalBrake() const
{
	bController->switchBehavior(normalBreak);
}

void FormationControlElement::switchIntoFlyBack() const
{
	bController->switchBehavior(flyBack);
}



