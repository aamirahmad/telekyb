/*
 * RoboCentricOmega6Joy.hpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#ifndef ROBOCENTRICOMEGA6JOY_HPP_
#define ROBOCENTRICOMEGA6JOY_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_behavior/Behavior.hpp>

#include <telekyb_base/Spaces/Angle.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// sensormsgs
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class RoboCentricOmega6Joy : public Behavior {
protected:
	Option<std::string>* tRoboCentricOmega6JoyTopic;
	Option<std::string>* tYawSinComponentTopic;
	Option<std::string>* tCommandedYawRateTopic;
	Option<bool>* tRoboCentricOmega6JoyUsePositionMode;
	Option<double>* tRoboCentricOmega6JoyYawRateScale;
	Option<double>* tSinPulse;
	Option<double>* tSinAmplitude;
	Option< std::string >* tVelocityInputTopic;

	//input is interpreted in local frame!
	Option<bool>* tRoboCentricOmega6JoyUseRelativeMode;
	//disable dead man switch?
	Option<bool>* tRoboCentricOmega6JoyUseDeadManSwitch;

	// ROS
	ros::NodeHandle nodeHandle;
	ros::Subscriber joySub;
	ros::Subscriber userInputSub;
	ros::Publisher yawSinComponentPublisher;
	ros::Publisher commandedYawRatePublisher;
	ros::Publisher velPhaseCorrectionPublisher;
	
	double currentTime;
	double lastTime;
	std_msgs::Float64 yawSinComponent;
	std_msgs::Float64 commandedYawRate;
	
	bool deadManPressed;
	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);
	void userVelocityCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

	Velocity3D lastVelocityInput;
	double lastYawRateInput;

	// Integrated Position for Velocity Mode
	Position3D posModeCurPosition;
	Angle posModeCurYawAngle;
	Time posModeLastInputTime;

	// Outputfield
	bool valid;

public:
	RoboCentricOmega6Joy();

	virtual void initialize();
	virtual void destroy();

	// Called directly after Change Event is registered.
	virtual bool willBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called after actual Switch. Note: During execution trajectoryStepCreation is used
	virtual void didBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called directly after Change Event is registered: During execution trajectoryStepTermination is used
	virtual void willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);
	// Called after actual Switch. Runs in seperate Thread.
	virtual void didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);

	// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
	virtual void trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
	virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const;
	double now();
};

}

#endif /* JOYSTICK_HPP_ */
