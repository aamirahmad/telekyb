#ifndef RCJOY_HPP_
#define RCJOY_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_behavior/Behavior.hpp>

#include <telekyb_base/Spaces/Angle.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// sensormsgs
#include <sensor_msgs/Joy.h>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class RCJoy : public Behavior {
protected:
	Option<std::string>* tRCJoyTopic;
	Option<bool>* tRCJoyUsePositionMode;
	Option<double>* tRCJoyYawRateScale;

	//input is interpreted in local frame!
	Option<bool>* tRCJoyUseRelativeMode;
	//disable dead man switch?
	Option<bool>* tRCJoyUseDeadManSwitch;

	// ROS
	ros::NodeHandle nodeHandle;
	ros::Subscriber joySub;

	void RCjoyCB(const sensor_msgs::Joy::ConstPtr& msg);

	Velocity3D lastVelocityInput;
	double lastYawRateInput;

	// Integrated Position for Velocity Mode
	Position3D posModeCurPosition;
	Angle posModeCurYawAngle;
	Time posModeLastInputTime;

	// Outputfield
	bool valid;

public:
	RCJoy();

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
	virtual void trajectoryStepActive(TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const;
};

}

#endif /* JOYSTICK_HPP_ */
