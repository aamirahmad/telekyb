/*
 * Experiment.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#ifndef EXPERIMENT_HPP_
#define EXPERIMENT_HPP_

#include <telekyb_base/Options.hpp>

#include <sensor_msgs/Joy.h>

#include <telekyb_interface/TeleKybCore.hpp>
#include <telekyb_interface/MKInterface.hpp>


using namespace telekyb;

class ExperimentOptions : public OptionContainer
{
public:
	Option<int>* robotID;
	Option<std::string>* tJoystickTopic;
	Option<bool>* tUseMKInterface;
	Option<bool>* tJoystickUseDeadManSwitch;
	Option<std::string>* tTrajPlaybackFilename;
	
	ExperimentOptions();
};

class Experiment : telekyb_interface::ActiveBehaviorListener {
protected:
	ExperimentOptions options;
	// ROS
	ros::NodeHandle mainNodeHandle;
	ros::Subscriber joySub;

	// the System
	telekyb_interface::TeleKybCore* core;
	// BehaviorController
	telekyb_interface::BehaviorController* bController;
	telekyb_interface::OptionController *oController;

	// Optional MKInterface
	telekyb_interface::MKInterface* mkInterface;


	// System Behaviors
	telekyb_interface::Behavior ground;
	telekyb_interface::Behavior hover;
	telekyb_interface::Behavior normalBreak;
	telekyb_interface::Behavior takeOff;
	telekyb_interface::Behavior land;

	// Custom
	telekyb_interface::Behavior joystick;
	telekyb_interface::Behavior trajPlayback;

	// Fly around three points.
	telekyb_interface::Behavior flyto1;
	telekyb_interface::Behavior flyto2;
	telekyb_interface::Behavior flyto3;

	// Behavior
	telekyb_interface::Behavior* activeBehaviorPtr;



	// setup Behaviors
	void setupExperiment();

public:
	Experiment();
	virtual ~Experiment();


	// From Interface
	void activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior);

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);
};

#endif /* EXPERIMENT_HPP_ */
