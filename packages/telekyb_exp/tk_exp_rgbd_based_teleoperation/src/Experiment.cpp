/*
 * Experiment.cpp
 *
 *  Created on: Jul 22, 2013
 *      Author: pstegagno
 */

#include "Experiment.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>

// Options
ExperimentOptions::ExperimentOptions()
	: OptionContainer("ExperimentOptions")
{
	//Experiment options
	robotID = addOption<int>("robotID", "Specify the robotID of the TeleKybCore to connect to.", 0, false, true);
	tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!", false, false, true);
	tInterruptTopic = addOption<std::string>("tInterruptTopic",
			"interrupt topic to use (std_msgs::Bool)", "/mkinterface_outdoor/humanOperator/interrupt", false, true);
	
	
	// ViconFreeLand options

	tAccComTopic = addOption<std::string>("tAccComTopic",
			"topic published by PositionControl",
			"comAcc", false, false);
	tVerticalVelTopic = addOption<std::string>("tVerticalVelTopic",
			"topic to get the commanded velocity", "undef", false, true);
	tDvoVelTopic = addOption<std::string>("tDvoVelTopic",
			"topic containing the DVO velocity estimate",
			"localState", false, false);
	tGainValue = addOption("tGainValue",
			"gain for the estimation of the accelaration", 12.0, false ,false);
	tFilterAlpha = addOption("tFilterAlpha",
			"alpha value used for the filter", 0.05, false ,false);
	tDetectionSensitivity = addOption("tDetectionSensitivity",
			"value used to estabilish the sensitivity to contacts", 3.0, false ,false);
	tWaitingTime = addOption("tWaitingTime",
			"seconds to wait before switching off the motors when no peaks are detected but the robot has landed", 1.5, false ,false);
	tLandVelocity = addOption<double>("tLandVelocity",
			"Defines the Velocity of the Land Behavior",
			0.5, false, false);
	
	// Omega6Joy options
	tOmega6JoyTopic = addOption<std::string>("tOmega6JoyTopic",
			"Omega6JoyTopictopic to use (sensor_msgs::Joy)", "/TeleKyb/tJoy/joy", false, true);
	tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);
	tCommandedYawRateTopic = addOption<std::string>("tCommandedYawRateTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);
	tYawSinComponentTopic = addOption<std::string>("tYawSinComponentTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);
	tSinPulse = addOption("tSinPulse",
			"Pulse for the yaw motion", 1.0, false ,false);
	tSinAmplitude = addOption("tSinAmplitude",
			"Amplitude for the yaw motion", 0.5, false ,false);

	
}


Experiment::Experiment()
	: mainNodeHandle( ROSModule::Instance().getMainNodeHandle() ), core(NULL), mkInterface(NULL)
{
	std::cout << "QUIIIIIIIIIIII A" << std::endl;
	core = telekyb_interface::TeleKybCore::getTeleKybCore(options.robotID->getValue());
	if (!core) {
		// fail
		std::cout << "QUIIIIIIIIIIII B" << std::endl;
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


	// sanity check
	if (ground.isNull() || hover.isNull() || normalBreak.isNull() || takeOff.isNull() ) {
		ROS_FATAL("Unable to get SystemBehavior!!!");
		//ROS_BREAK();
		ros::shutdown();
	}
	
	// done
	takeOff.setParameterInitialized(true);
	
	land = bController->loadBehavior("tk_be_common/ViconFreeLand");
	
	Omega6Joy = bController->loadBehavior("tk_be_common/RoboCentricOmega6Joy");

	if ( land.isNull() ) {
		ROS_FATAL("Unable to load ViconFreeLand!!!");
		//ROS_BREAK();
		ros::shutdown();
	}
	
	if ( Omega6Joy.isNull() ) {
		ROS_FATAL("Unable to load Omega6Joy!!!");
		//ROS_BREAK();
		ros::shutdown();
	}
	
	// setup ViconFreeLand
	land.getOptionContainer().getOption("tAccComTopic").set(options.tAccComTopic->getValue());
	land.getOptionContainer().getOption("tVerticalVelTopic").set(options.tVerticalVelTopic->getValue());
	land.getOptionContainer().getOption("tDvoVelTopic").set(options.tDvoVelTopic->getValue());
	land.getOptionContainer().getOption("tGainValue").set(options.tGainValue->getValue());
	land.getOptionContainer().getOption("tFilterAlpha").set(options.tFilterAlpha->getValue());
	land.getOptionContainer().getOption("tDetectionSensitivity").set(options.tDetectionSensitivity->getValue());
	land.getOptionContainer().getOption("tWaitingTime").set(options.tWaitingTime->getValue());
	land.getOptionContainer().getOption("tLandVelocity").set(options.tLandVelocity->getValue());
	land.setParameterInitialized(true);
	land.setNextBehavior(ground);
	
	// setup Omega6Joy
	Omega6Joy.getOptionContainer().getOption("tSinPulse").set(options.tSinPulse->getValue());
	Omega6Joy.getOptionContainer().getOption("tSinAmplitude").set(options.tSinAmplitude->getValue());
	Omega6Joy.getOptionContainer().getOption("tOmega6JoyTopic").set(options.tOmega6JoyTopic->getValue());
	Omega6Joy.getOptionContainer().getOption("tVelocityInputTopic").set(options.tVelocityInputTopic->getValue());
	Omega6Joy.getOptionContainer().getOption("tCommandedYawRateTopic").set(options.tCommandedYawRateTopic->getValue());
	Omega6Joy.getOptionContainer().getOption("tYawSinComponentTopic").set(options.tYawSinComponentTopic->getValue());
	Omega6Joy.setParameterInitialized(true);

	// flyto1 = bController->loadBehavior("tk_be_common/FiltFlyTo");
	// flyto1.getOptionContainer().getOption("tFlyToDestination").set(Position3D(0.0,0.0,-1.0));
	// flyto1.setParameterInitialized(true);
	// flyto1.setNextBehavior(Omega6Joy);

	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV not in Ground Behavior during Startup");
		ros::shutdown();
	}

	// lastly start Controller
	joySub = mainNodeHandle.subscribe(options.tOmega6JoyTopic->getValue()
			, 10, &Experiment::Omega6JoyCB, this);
	interrupt = mainNodeHandle.subscribe(options.tInterruptTopic->getValue()
			, 10, &Experiment::Omega6JoyInterruptCB, this);
	
	
	interrupted = false;
}




void Experiment::Omega6JoyCB(const sensor_msgs::Joy::ConstPtr& msg)
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

	
	if (!interrupted) {
	
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
					} else {
						ROS_ERROR("Motors have to be on for liftOff!");
						return;
					}
				}
				else {
				    bController->switchBehavior(takeOff);
				}

				// takeoff
// 				
			} else {
				// flying -> land
				bController->switchBehavior(land);
			}
		}

		if (msg->buttons[3]) {
			bController->switchBehavior(Omega6Joy);
		}
	}
}


void Experiment::Omega6JoyInterruptCB(const std_msgs::Bool::ConstPtr& msg)
{
	if (interrupted && !msg->data){
		bController->switchBehavior(flyto1);
	}
	interrupted = msg->data;
// 	std::cout << "  interrupted   " << interrupted << std::endl;
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

