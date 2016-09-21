/*
 * Experiment.hpp
 *
 *  Created on: Jul 22, 2013
 *      Author: pstegagno
 */

#ifndef EXPERIMENT_HPP_
#define EXPERIMENT_HPP_

#include <telekyb_base/Options.hpp>

#include <std_msgs/Bool.h>
#include <telekyb_msgs/TKState.h>
#include <rosgraph_msgs/Clock.h>

#include <mav_msgs/CommandTrajectory.h>

#include <telekyb_interface/TeleKybCore.hpp>


using namespace telekyb;

class ExperimentOptions : public OptionContainer
{
public:
	Option<int>* robotID;
	Option<std::string>* tOmega6JoyTopic;
	Option<std::string>* tInterruptTopic;
	Option<std::string>* tVelocityInputTopic;
	Option<std::string>* tCommandedYawRateTopic;
	Option<std::string>* tYawSinComponentTopic;
	Option<double>* tSinPulse;
	Option<double>* tSinAmplitude;
	Option<bool>* tUseMKInterface;
	ExperimentOptions();
};

class Experiment : telekyb_interface::ActiveBehaviorListener {
protected:
	ExperimentOptions options;
	// ROS
	ros::NodeHandle mainNodeHandle;
	
	ros::Subscriber tkstatesub;
	ros::Subscriber waypointsub;

	// the System
	telekyb_interface::TeleKybCore* core;
	// BehaviorController
	telekyb_interface::BehaviorController* bController;
	telekyb_interface::OptionController *oController;

	// System Behaviors
	telekyb_interface::Behavior ground;
	telekyb_interface::Behavior hover;
	telekyb_interface::Behavior normalBreak;
	telekyb_interface::Behavior takeOff;
	telekyb_interface::Behavior land;

	// Custom
	telekyb_interface::Behavior RCjoy;
	telekyb_interface::Behavior trajPlayback;

	// Fly around three points.
	telekyb_interface::Behavior flyto1;

	// Behavior
	telekyb_interface::Behavior* activeBehaviorPtr;
	
	
	bool interrupted;
	bool startingpositionreached;




public:
	Experiment();
	virtual ~Experiment();

	// setup Behaviors
	void setupExperiment();

	// From Interface
	void activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior);

	void clockCB(const rosgraph_msgs::Clock::ConstPtr& msg);
	void waypointCB(const mav_msgs::CommandTrajectory::ConstPtr& msg);
	
};

#endif /* EXPERIMENT_HPP_ */
