/*
 * TeleopPoint.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: eruff
 */

#ifndef FIXED_POINT_HOVER_HPP_
#define FIXED_POINT_HOVER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_behavior/Behavior.hpp>

// Options
#include <telekyb_base/Options.hpp>
// sensormsgs
#include <sensor_msgs/Joy.h>
// plugin stuff
#include <pluginlib/class_list_macros.h>


#include <telekyb_base/TeleKyb.hpp>

#include <telekyb_base/Spaces/Angle.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class TeleopPoint : public Behavior {
protected:
	// Option
	Option<std::string>* tJoystickTopic;
	Option<double>* tTeleopPointMaxInitialVelocity;
	Option<bool>* tTeleopPointVelocityMode;
	Option<Position3D>* tTeleopPointStartingPosition;
	Option<double>* tTeleopPointScalingThrust;
	Option<double>* tTeleopPointScalingRoll;
	Option<double>* tTeleopPointScalingPitch;
	Option<double>* tTeleopPointScalingYaw;

    Option<double>* tTeleopPointScalingZ;
    Option<double>* tTeleopPointScalingY;
    Option<double>* tTeleopPointScalingX;
    Option<double>* tTeleopPointScalingYawDot;


	// ROS
	ros::NodeHandle nodeHandle;
	ros::Subscriber _joySub;
	
	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);

	void updatePosition();
    void updateVelocity();
	// Data
	Position3D _position;
    Vector3D _velocity;
    double _yawRate;
	double _yawAngle; // explicit, to do only 1 RPY Conversion
	sensor_msgs::Joy _joyMsg;
	bool _joyMsgCalledbackOnce;
	// Init Service
	//ros::ServiceServer initService;
  
	bool valid;
public:
	TeleopPoint();

	// from BehaviorInterface
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

	// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or TeleopPointHover if undef).
	virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const;

    virtual void setTrajectoryHeader(TKTrajectory& generatedTrajInput);

};

}

#endif /* HOVERBEHAVIOR_HPP_ */
