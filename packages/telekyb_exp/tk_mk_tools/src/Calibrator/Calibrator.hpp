/*
 * Calibrator.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#ifndef CALIBRATOR_HPP_
#define CALIBRATOR_HPP_

#include <telekyb_base/Options.hpp>

#include <sensor_msgs/Joy.h>

#include <telekyb_interface/TeleKybCore.hpp>
#include <telekyb_interface/MKInterface.hpp>


using namespace telekyb;

class CalibratorOptions : public OptionContainer
{
public:
	Option<int>* robotID;
	Option<std::string>* tJoystickTopic;
	Option<bool>* tUseMKInterface;
	CalibratorOptions();
};

class Calibrator : telekyb_interface::ActiveBehaviorListener {
protected:
	CalibratorOptions options;
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

	// Calibrator
	telekyb_interface::Behavior calibrator;

	// Behavior
	telekyb_interface::Behavior* activeBehaviorPtr;

	//double saveIntegralGain;




public:
	Calibrator();
	virtual ~Calibrator();

	// setup Behaviors
	void setupCalibrator();

	void activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior);

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);
};

#endif /* CALIBRATOR_HPP_ */
