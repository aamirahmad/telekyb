/*
 * TrajectoryInput.cpp
 *
 *  Created on: Mar 2, 2012
 *      Author: mriedel
 */

#include "TrajectoryInput.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKTrajectory.h>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::TrajectoryInput, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

TrajectoryInput::TrajectoryInput()
	: Behavior("tk_be_common/TrajectoryInput", BehaviorType::Air)
{

}


TrajectoryInput::~TrajectoryInput()
{

}

void TrajectoryInput::trajectoryCB(const telekyb_msgs::TKTrajectory::ConstPtr& msg)
{
	timeoutTimer.reset();
	// TODO: protect by Mutex
	lastMsg.fromTKTrajMsg(*msg);
}

void TrajectoryInput::initialize()
{
	ROS_WARN("Initialized TrajectoryInput Behavior.");
	tTrajectoryTopicName = addOption<std::string>("tTrajectoryTopicName",
			"tTrajectoryTopicName that published telekyb_msgs::TKTrajectory",
			"custom", false, false);
	tTrajectoryTimeout = addOption<double>("tTrajectoryTimeout",
			"Timeout. Behavior turns invalid if it does not recv new message within timeout.",
			1.0, false, false);

	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	// no Parameters
	//parameterInitialized = true;
}

void TrajectoryInput::destroy()
{

}

bool TrajectoryInput::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// Initialize Message to 0 Velocity and 0 YawRate
	lastMsg.setVelocity(Velocity3D::Zero());
	lastMsg.setYawRate(0.0);

	trajectorySub = nodeHandle.subscribe(tTrajectoryTopicName->getValue()
			, 10, &TrajectoryInput::trajectoryCB, this);

	timeoutTimer.reset();
	return true;
}

void TrajectoryInput::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void TrajectoryInput::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void TrajectoryInput::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	trajectorySub.shutdown();
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void TrajectoryInput::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void TrajectoryInput::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	//	ROS_INFO_STREAM("InputVel: " << std::endl << lastVelocityInput);
	//	ROS_INFO_STREAM("InputYawRate: " << lastYawRateInput);

	// TODO: protect by Mutex
	generatedTrajInput = lastMsg;
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void TrajectoryInput::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool TrajectoryInput::isValid(const TKState& currentState) const
{
	return timeoutTimer.getElapsed().toDSec() < tTrajectoryTimeout->getValue();;
}


}
