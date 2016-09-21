/*
 * Experiment.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#ifndef EXPERIMENT_HPP_
#define EXPERIMENT_HPP_

#include <telekyb_base/Options.hpp>

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Messages/TKState.hpp>

#include <sensor_msgs/Joy.h>

#include <telekyb_interface/TeleKybCore.hpp>
#include <telekyb_interface/MKInterface.hpp>


using namespace TELEKYB_NAMESPACE;

class ExperimentOptions : public OptionContainer
{
public:
	Option<int>* robotID;
	Option<std::string>* tJoystickTopic;
	Option<bool>* tUseMKInterface;
	Option<std::string>* tExternalTrajectoryTopic;
	Option<Position3D>* tTakeOffDestination;
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
	telekyb_interface::Behavior trajForceInput;

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
