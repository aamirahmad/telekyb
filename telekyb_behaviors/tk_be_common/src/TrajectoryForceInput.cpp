/*
 * TrajectoryForceInput.cpp
 *
 *  Created on: Mar 2, 2012
 *      Author: mriedel
 */

#include "TrajectoryForceInput.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKTrajectory.h>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::TrajectoryForceInput, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

TrajectoryForceInput::TrajectoryForceInput()
	: Behavior("tk_be_common/TrajectoryForceInput", BehaviorType::Air)
{

}


TrajectoryForceInput::~TrajectoryForceInput()
{

}

void TrajectoryForceInput::trajectoryCB(const geometry_msgs::PoseWithCovariance::ConstPtr& msg)
{
	timeoutTimer.reset();
	// TODO: protect by Mutex
	lastMsg.jerk[0] = msg->covariance[9];
	lastMsg.jerk[1] = msg->covariance[10];
	lastMsg.jerk[2] = msg->covariance[11];
	
	lastMsg.acceleration[0] = msg->covariance[6];
	lastMsg.acceleration[1] = msg->covariance[7];
	lastMsg.acceleration[2] = msg->covariance[8];

	lastMsg.velocity[0] = msg->covariance[3];
	lastMsg.velocity[1] = msg->covariance[4];
	lastMsg.velocity[2] = msg->covariance[5];
	
	lastMsg.setPosition(Position3D(msg->covariance[0], msg->covariance[1], msg->covariance[2]));

}

void TrajectoryForceInput::initialize()
{
	ROS_WARN("Initialized TrajectoryForceInput Behavior.");
	tTrajectoryTopicName = addOption<std::string>("tTrajectoryTopicName",
			"tTrajectoryTopicName that published telekyb_msgs::TKTrajectory",
			"ExtTraj", false, false);
	tTrajectoryTimeout = addOption<double>("tTrajectoryTimeout",
			"Timeout. Behavior turns invalid if it does not recv new message within timeout.",
			1.0, false, false);

	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	// no Parameters
	//parameterInitialized = true;
}

void TrajectoryForceInput::destroy()
{

}

bool TrajectoryForceInput::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// Initialize Message to 0 Velocity and 0 YawRate
	
	lastMsg.jerk = Velocity3D::Zero();
	lastMsg.acceleration = Velocity3D::Zero();
	lastMsg.snap = Velocity3D::Zero();
	
	lastMsg.setVelocity(Velocity3D::Zero());
	lastMsg.setYawRate(0.0);

	trajectorySub = nodeHandle.subscribe(tTrajectoryTopicName->getValue()
			, 10, &TrajectoryForceInput::trajectoryCB, this);

	timeoutTimer.reset();
	return true;
}

void TrajectoryForceInput::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void TrajectoryForceInput::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void TrajectoryForceInput::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	trajectorySub.shutdown();
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void TrajectoryForceInput::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void TrajectoryForceInput::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	//	ROS_INFO_STREAM("InputVel: " << std::endl << lastVelocityInput);
	//	ROS_INFO_STREAM("InputYawRate: " << lastYawRateInput);

	// TODO: protect by Mutex
	generatedTrajInput = lastMsg;
	generatedTrajInput.setYawAngle( 0.0 );
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void TrajectoryForceInput::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool TrajectoryForceInput::isValid(const TKState& currentState) const
{
	return timeoutTimer.getElapsed().toDSec() < tTrajectoryTimeout->getValue();;
}


}
