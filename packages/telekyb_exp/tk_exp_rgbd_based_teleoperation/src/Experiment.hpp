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
#include <sensor_msgs/Joy.h>

#include <telekyb_interface/TeleKybCore.hpp>
#include <telekyb_interface/MKInterface.hpp>


using namespace telekyb;

class ExperimentOptions : public OptionContainer
{
public:
	Option<int>* robotID;
	Option<std::string>* tInterruptTopic;
	Option<bool>* tUseMKInterface;
	
	// Omega6Joy options 
	Option<std::string>* tOmega6JoyTopic;
	Option<std::string>* tVelocityInputTopic;
	Option<std::string>* tCommandedYawRateTopic;
	Option<std::string>* tYawSinComponentTopic;
	Option<double>* tSinPulse;
	Option<double>* tSinAmplitude;
	
	// ViconFreeLand Options
	Option<double>* tLandVelocity;
	Option<std::string>* tAccComTopic;
	Option<std::string>* tDvoVelTopic;
	Option<std::string>* tVerticalVelTopic;
	Option<double>* tGainValue;
	Option<double>* tWaitingTime;
	Option<double>* tFilterAlpha;
	Option<double>* tDetectionSensitivity;
	
	ExperimentOptions();
};

class Experiment : telekyb_interface::ActiveBehaviorListener {
protected:
	ExperimentOptions options;
	// ROS
	ros::NodeHandle mainNodeHandle;
	ros::Subscriber joySub;
	ros::Subscriber interrupt;

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
	telekyb_interface::Behavior Omega6Joy;

	// Fly around three points.
	telekyb_interface::Behavior flyto1;

	// Behavior
	telekyb_interface::Behavior* activeBehaviorPtr;
	
	
	bool interrupted;




public:
	Experiment();
	virtual ~Experiment();

	// setup Behaviors
	void setupExperiment();

	// From Interface
	void activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior);

	void Omega6JoyCB(const sensor_msgs::Joy::ConstPtr& msg);
	
	void Omega6JoyInterruptCB(const std_msgs::Bool::ConstPtr& msg);
	
};

#endif /* EXPERIMENT_HPP_ */
