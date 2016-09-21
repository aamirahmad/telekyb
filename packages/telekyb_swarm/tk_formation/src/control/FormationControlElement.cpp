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
		mkInterface = telekyb_interface::MKInterface::getMKInterface(robotID, 5.0);
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
	//takeOff.getOptionContainer().getOption("tTakeOffDestination").set(Position3D(0.0,0.0,-1.0));
	takeOff.getOptionContainer().getOption("tTakeOffVertically").set(false);

	// done
	takeOff.setParameterInitialized(true);

	//land.getOptionContainer().getOption("tLandDestination").set(Position3D(0.0,0.0,0.0));
	land.getOptionContainer().getOption("tLandVertically").set(true);
	land.setParameterInitialized(true);


	formation4 = bController->loadBehavior("tk_formation/Formation");
	formation3 = bController->loadBehavior("tk_formation/Formation");

	if ( formation4.isNull() || formation3.isNull()) {
		ROS_FATAL("Unable to load FormationControl for UAV %d !!!", robotID);
		//ROS_BREAK();
		ros::shutdown();
	}


	tetraeder = bController->loadBehavior("tk_formation/FormationReconfiguration");
	square = bController->loadBehavior("tk_formation/FormationReconfiguration");
	triangle = bController->loadBehavior("tk_formation/FormationReconfiguration");
	if ( tetraeder.isNull() || square.isNull() || triangle.isNull() ) {
		ROS_FATAL("Unable to load FormationReconfiguration Behavior for UAV %d !!!", robotID);
		//ROS_BREAK();
		ros::shutdown();
	}

	// configure calibrator
	//formation.setNextBehavior(land);
	//formation.setParameterInitialized(true);


	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV %d not in Ground Behavior during Startup", robotID);
		ros::shutdown();
	}
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
	if (mkInterface && newActiveBehavior == ground) {
		MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
		motorState.value = MotorState::Off;
		mkInterface->setMKValue(motorState);
	}
}

//telekyb_interface::Behavior FormationControlElement::getFormationBehavior() const
//{
//	return formation;
//}

//void FormationControlElement::switchIntoFormation() const
//{
//	bController->switchBehavior(formation);
//}






