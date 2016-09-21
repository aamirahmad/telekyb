/*
 * Joystick.hpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#ifndef JOYSTICK_HPP_
#define JOYSTICK_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_behavior/Behavior.hpp>

#include <telekyb_base/Spaces/Angle.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// sensormsgs
#include <sensor_msgs/Joy.h>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class Joystick : public Behavior {
protected:
	Option<std::string>* tJoystickTopic;
	Option<bool>* tJoystickUsePositionMode;
	Option<double>* tJoystickYawRateScale;


    Option<double>* tJoystickZScale;
    Option<double>* tJoystickYScale;
    Option<double>* tJoystickXScale;

    Option<int>* tJoystickCommand_FORBACK;
    Option<int>* tJoystickCommand_LEFTRIGHT;
    Option<int>* tJoystickCommand_UPDOWN;
    Option<int>* tJoystickCommand_YAW;
    Option<int>* tJoystickCommand_DISABLE;
    Option<int>* tJoystickCommand_DEADMANSWITCH;

	//input is interpreted in local frame!
	Option<bool>* tJoystickUseRelativeMode;
	//disable dead man switch?
	Option<bool>* tJoystickUseDeadManSwitch;

    sensor_msgs::Joy _joyMsg;

	// ROS
	ros::NodeHandle nodeHandle;
	ros::Subscriber joySub;

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);


    Position3D _position;
    Vector3D _velocity;
    double _yawRate;
    double _yawAngle; // explicit, to do only 1 RPY Conversion

	Velocity3D lastVelocityInput;
	double lastYawRateInput;

	// Integrated Position for Velocity Mode
	Position3D posModeCurPosition;
	Angle posModeCurYawAngle;
	Time posModeLastInputTime;

	// Outputfield
	bool valid;

    void updateVelocity();
    void updatePosition();

public:
	Joystick();

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
};

}

#endif /* JOYSTICK_HPP_ */
