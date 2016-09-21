/**
 * TeleopExperiment.hpp
 *
 * Distributed under the Boost Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or
 * copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 *  Created on: Sep 29, 2014
 *      Author: Johannes Lächele
 *  
 */
#ifndef TELEOPEXPERIMENT_HPP_
#define TELEOPEXPERIMENT_HPP_

#include <telekyb_base/Time.hpp>

#include <boost/asio.hpp>
#include <Eigen/Eigen>

#include <telekyb_msgs/TKState.h>
#include <sensor_msgs/Joy.h>

#include <telekyb_base/Options.hpp>
#include <telekyb_interface/TeleKybCore.hpp>
#include <telekyb_interface/MKInterface.hpp>

using namespace telekyb;
using boost::asio::ip::udp;

class TeleopExperimentOptions : public OptionContainer {
public:
	Option<int>* robotID;
	Option<std::string>* tJoystickTopic;
	Option<std::string>* tWittensteinTopic;

	Option<std::string>* tTKStateTopic;
	Option<std::string>* tCMSControllerHost;
	Option<std::string>* tCMSControllerPort;

	TeleopExperimentOptions();
};

class TeleopExperiment : telekyb_interface::ActiveBehaviorListener {
	/*
	 * name of host and port where to send the state of the Octo
	 */
	boost::asio::io_service io_service;
	udp::resolver resolver;
	udp::endpoint mStateEndpoint;
	udp::socket mStateSocket;

	ros::Subscriber mStateSub;
	Eigen::VectorXd vehicleState;
	Eigen::VectorXd targetState;
	double mTriggers;

	void sendStatepackage ();
protected:
	TeleopExperimentOptions options;

	// ROS
	ros::NodeHandle mainNodeHandle;

	// the System
	telekyb_interface::TeleKybCore* core;
	// BehaviorController
	telekyb_interface::BehaviorController* bController;
	telekyb_interface::OptionController *oController;

	// Behavior
	telekyb_interface::Behavior* activeBehaviorPtr;
	// System Behaviors
	telekyb_interface::Behavior ground;
	telekyb_interface::Behavior hover;
	telekyb_interface::Behavior normalBreak;
	telekyb_interface::Behavior takeOff;
	telekyb_interface::Behavior land;
	// Custom
	telekyb_interface::Behavior teleop;
	telekyb_interface::Behavior flytoStartPosition;

	ros::Subscriber joySub;
	ros::Subscriber WSSub;
	// MKInterface
	telekyb_interface::MKInterface* mkInterface;

	// setup Behaviors
	void setupExperiment();

	//measures when participants moved to target after completing sideways motion.
	bool movedLeft, movedRight, crossedcenter;
	double distanceLeftRight;
	Time startTime;
public:
	TeleopExperiment();
	virtual ~TeleopExperiment();

	// From Interface
	void activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior);

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);
	void WSCB(const sensor_msgs::Joy::ConstPtr& msg);
	void StateCB(const telekyb_msgs::TKState& msg);
};

#endif /* TELEOPEXPERIMENT_HPP_ */
