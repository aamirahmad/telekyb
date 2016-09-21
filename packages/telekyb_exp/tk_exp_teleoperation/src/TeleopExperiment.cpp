/**
 * TeleopExperiment.cpp
 *
 * Distributed under the Boost Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or
 * copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 *  Created on: Sep 29, 2014
 *      Author: Johannes Lächele
 *  
 */
#include "TeleopExperiment.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>
#include <telekyb_base/Messages.hpp>

#include <telekyb_base/Spaces.hpp>

// Options
TeleopExperimentOptions::TeleopExperimentOptions() : OptionContainer("ExperimentOptions")
{
	robotID = addOption<int>("robotID", "Specify the robotID of the TeleKybCore to connect to.", 0, false, true);
	tJoystickTopic = addOption<std::string>("tJoystickTopic", "Joysticktopic to use (sensor_msgs::Joy)", "/TeleKyb/tJoy/joy", false, true);
	tWittensteinTopic = addOption<std::string>("tWittensteinTopic", "Topic of the wittenstein receiver", "/TeleKyb/tJoy/Wittensteins", true, true);

	tTKStateTopic = addOption<std::string>("tTKStateTopic", "Name of the TKState topic", "/TeleKyb/TeleKybSystem_20/Sensor/TKState", true, true);
	tCMSControllerHost = addOption<std::string>("tCMSControllerHost", "Hostname/IP of the receiver of the octorotor state that drives the CMS", "192.168.44.102", true, true);
	tCMSControllerPort = addOption<std::string>("tCMSControllerPort", "Port of the receiver of the octorotor state that drives the CMS", "4568", true, true);
}

TeleopExperiment::TeleopExperiment() : mainNodeHandle( ROSModule::Instance().getMainNodeHandle() ),
		core(NULL),
		mkInterface(NULL),
		resolver(io_service),
		vehicleState(18),
		targetState(18),
		mStateSocket(io_service),
		movedLeft(false),
		movedRight(false),
		crossedcenter(false),
		distanceLeftRight(2)
{
	core = telekyb_interface::TeleKybCore::getTeleKybCore(options.robotID->getValue());
	if (!core) {
		// fail
		ros::shutdown();
		return;
	}

	//options.tUseMKInterface->setValue(true);
	ROS_INFO("Creating MKInterface!");
	// use MKInterface
	mkInterface = telekyb_interface::MKInterface::getMKInterface(options.robotID->getValue());
	//mkInterface = telekyb_interface::MKInterface::getMKInterface(1); // BEWARE TEMPORARY!!!
	if (!mkInterface) {
		// fail
		ros::shutdown();
		return;
	}

	bController = core->getBehaviorController();
	oController = core->getOptionController();

	//activeBehavior = bController->getActiveBehaviorReference();
	bController->setActiveBehaviorListener(this);

	activeBehaviorPtr = bController->getActiveBehaviorPointer();

	//get the host/port names to where the received wittenstein commands will be sent to
	udp::resolver::query query(udp::v4(), options.tCMSControllerHost->getValue(), options.tCMSControllerPort->getValue());
	mStateEndpoint = *resolver.resolve(query);
	mStateSocket.open(udp::v4());

	//we start at the ground, right? :-)
	mTriggers = 10;
	mStateSub = mainNodeHandle.subscribe(options.tTKStateTopic->getValue(), 1, &TeleopExperiment::StateCB, this);

	setupExperiment();
}

TeleopExperiment::~TeleopExperiment() {
}

void TeleopExperiment::StateCB(const telekyb_msgs::TKState& msg) {
	TKState state;
	state.fromTKStateMsg(msg);
	Vector3D euler = state.getEulerRPY();
	Vector3D eulerdot = state.getEulerDotRPY();

	//seems to be working fine (wrt world frame)
//	std::cout << "euler: " << (double)euler(0) << "\t" << (double)euler(1) << "\t" << (double)euler(2) << std::endl;

	//seems to be working fine (wrt local frame!)
//	std::cout << "eulerdot: " << (double)eulerdot(0) << "\t" << (double)eulerdot(1) << "\t" << (double)eulerdot(2) << std::endl;
	/*
	 * vehicle state	18
	 * target state		18
	 * triggers			1
	 *
	 */
	vehicleState.fill(0);
	targetState.fill(0);

	vehicleState(0) = msg.pose.position.x;
	vehicleState(1) = msg.pose.position.y;
	vehicleState(2) = msg.pose.position.z;
	vehicleState(3) = msg.twist.linear.x;
	vehicleState(4) = msg.twist.linear.y;
	vehicleState(5) = msg.twist.linear.z;
	vehicleState(6) = 0;
	vehicleState(7) = 0;
	vehicleState(8) = 0;

	//working fine (wrt world frame)
	//std::cout << "pos: " << (double)vehicleState(0) << "\t" << (double)vehicleState(1) << "\t" << (double)vehicleState(2) << std::endl;

	//working fine (wrt world frame)
//	std::cout << "vel: " << (double)vehicleState(3) << "\t" << (double)vehicleState(4) << "\t" << (double)vehicleState(5) << std::endl;

	vehicleState(9) = euler(0);
	vehicleState(10) = euler(1);
	vehicleState(11) = euler(2);
	vehicleState(12) = eulerdot(0);
	vehicleState(13) = eulerdot(1);
	vehicleState(14) = eulerdot(2);
	vehicleState(15) = 0;
	vehicleState(16) = 0;
	vehicleState(17) = 0;

	targetState(0) = 0;//lateral goal position, the other values will not be used!
	targetState(1) = 2;
	targetState(2) = -2;

	double send_buf[37];
	for (unsigned i = 0; i < 18; i++) {
		send_buf[i] = vehicleState(i);
		send_buf[i+18] = targetState(i);
	}
	send_buf[36] = mTriggers;
	mStateSocket.send_to(boost::asio::buffer(send_buf), mStateEndpoint);

	if (*activeBehaviorPtr == teleop) {
		if ((vehicleState(0) - targetState(0)) > distanceLeftRight) {
			movedLeft = true;
			mTriggers = 62;
		} else if ((targetState(0) - vehicleState(0)) > distanceLeftRight) {
			movedRight = true;
			mTriggers = 64;
		}

		if (movedLeft && movedRight && !crossedcenter) {
			if (std::fabs((double)(vehicleState(0) - targetState(0))) < 0.05) {
				crossedcenter = true;
				mTriggers = 66; //reached center again
				startTime = Time();
			}
		}

		if (crossedcenter && ((Time() - startTime).toDSec() > 20)) {
			bController->switchBehavior(flytoStartPosition);
			//reset conditions!
			movedLeft = false;
			movedRight = false;
			crossedcenter = false;
			mTriggers = 30;
		}
	}

}

void TeleopExperiment::setupExperiment() {
	movedLeft = false;
	movedRight = false;
	crossedcenter = false;
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
	//	takeOff.getOptionContainer().getOption("tTakeOffDestination").set(Position3D(0.0,0.0,-1.0));
	takeOff.getOptionContainer().getOption("tTakeOffVertically").set(true);

	// done
	takeOff.setParameterInitialized(true);

	land.getOptionContainer().getOption("tLandVelocity").set<double>(0.4);
	land.getOptionContainer().getOption("tLandDestination").set(Position3D(0.0,0.0,0.0));
	//	land.getOptionContainer().getOption("tLandDestinationHeight").set<double>(-0.25);
	land.setParameterInitialized(true);

	teleop = bController->loadBehavior("tk_be_common/Teleoperation");
	if ( teleop.isNull() ) {
		ROS_FATAL("Unable to load Teleoperation behavior!!!");
		//ROS_BREAK();
		ros::shutdown();
	}
	// setup teleop
	teleop.getOptionContainer().getOption("tJoystickTopic").set(options.tJoystickTopic->getValue());
	teleop.setParameterInitialized(true);

	flytoStartPosition = bController->loadBehavior("tk_be_common/LinearFlyTo");
	flytoStartPosition.getOptionContainer().getOption("tFlyToDestination").set(Position3D(0,2,-2.0));
	flytoStartPosition.getOptionContainer().getOption("tFlyToVelocity").set(0.5);
	flytoStartPosition.setParameterInitialized(true);

	takeOff.setNextBehavior(flytoStartPosition);
	teleop.setNextBehavior(flytoStartPosition);

	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV not in Ground Behavior during Startup");
		ros::shutdown();
	}

	// finally start Controller
	joySub = mainNodeHandle.subscribe(options.tJoystickTopic->getValue(), 10, &TeleopExperiment::joystickCB, this);
	WSSub = mainNodeHandle.subscribe(options.tWittensteinTopic->getValue(), 1, &TeleopExperiment::WSCB, this);

}

void TeleopExperiment::WSCB(const sensor_msgs::Joy::ConstPtr& msg) {
	if (msg->buttons[0] > 0 && *activeBehaviorPtr == hover) {
		ROS_INFO("switching to teleop behavior!");
		bController->switchBehavior(teleop);
	}
}

void TeleopExperiment::joystickCB(const sensor_msgs::Joy::ConstPtr& msg) {

	// use button 2
	if (msg->buttons.size() < 9) {
		ROS_ERROR("Joystick does not publish enough buttons.");
		return;
	}

	// Emergency
	if (msg->buttons[6]) {
		ROS_WARN("Emergency Button pressed!");
		mkInterface->setEmergency();
		mTriggers = -100;
	}

	// Button 1! toggle Motors for mkInterface Only in Ground
	if (*activeBehaviorPtr == ground && msg->buttons[0]) {
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

	//Button made for take-off
	if (msg->buttons[2]) {
		if (*activeBehaviorPtr == ground) {

			MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
			if (!mkInterface->updateMKValue(motorState)) {
				ROS_ERROR("Could not get motor state for take-off!");
				return;
			}
			if (motorState.value == MotorState::On) {
				bController->switchBehavior(takeOff);
			} else {
				ROS_ERROR("Motors have to be on for take-off!");
			}
			return;
		} else {
			// flying -> land
			bController->switchBehavior(land);
		}
	}

	if (msg->buttons[3] && *activeBehaviorPtr == hover) {
		//TODO:
//		bController->switchBehavior(teleop);
	}

}

void TeleopExperiment::activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior) {
	// Automatically turn off Motors
	if (newActiveBehavior == ground) {
		MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
		motorState.value = MotorState::Off;
		mkInterface->setMKValue(motorState);
	}

	if (mTriggers < 0) {
		//emergency state, do not allow state change to something else.
		return;
	}

	if (newActiveBehavior == ground) {
		mTriggers = 10;
	} else if (newActiveBehavior == takeOff) {
		mTriggers = 20;
	} else if (newActiveBehavior == flytoStartPosition) {
		mTriggers = 30;
	} else if (newActiveBehavior == normalBreak) {
		mTriggers = 40;
	} else if (newActiveBehavior == hover) {
		mTriggers = 50;
	} else if (newActiveBehavior == teleop) {
		mTriggers = 60;
	} else {
		mTriggers = 0;
	}

	std::cout << mTriggers << std::endl;
}
