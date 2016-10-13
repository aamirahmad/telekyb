/*
 * LinearFlyTo.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: mriedel
 */

#include "LinearFlyTo.hpp"

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::LinearFlyTo, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

LinearFlyTo::LinearFlyTo()
	: Behavior("tk_be_common/LinearFlyTo", BehaviorType::Air)
{

}

void LinearFlyTo::initialize()
{
	tFlyToDestination = addOption<Position3D>("tFlyToDestination",
			"Specifies the Destination of the LinearFlyTo Behavior",
			Position3D(0.0,0.0,-1.0), false, false);
	tFlyToVelocity = addOption<double>("tFlyToVelocity",
			"Defines the Velocity of the LinearFlyTo Behavior",
			1.0, false, false);
	tFlyToDestinationRadius = addOption<double>("tFlyToDestinationRadius",
			"Defines the Radius from the Destination, at which the LinearFlyTo Behavior turns invalid",
			0.25, false, false);

	//parameterInitialized = true; // not parameter Initialized by default.
}

void LinearFlyTo::destroy()
{

}

bool LinearFlyTo::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	yawAngle = currentState.getEulerRPY()(2);
	//ROS_INFO("LinearFlyTo: Storing yawAngle: %f ", yawAngle);
	return true;
}

void LinearFlyTo::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void LinearFlyTo::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void LinearFlyTo::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void LinearFlyTo::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	trajectoryStepActive(currentState, generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void LinearFlyTo::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	Vector3D direction = tFlyToDestination->getValue() - currentState.position;
	generatedTrajInput.setVelocity( direction.normalized() * tFlyToVelocity->getValue() );
	generatedTrajInput.setYawAngle( yawAngle );
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void LinearFlyTo::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawAngle( yawAngle );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool LinearFlyTo::isValid(const TKState& currentState) const
{
	Vector3D direction = tFlyToDestination->getValue() - currentState.position;
	return direction.norm() > tFlyToDestinationRadius->getValue();
}


} /* namespace telekyb_behavior */
