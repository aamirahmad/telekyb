/*
 * DynamicFlyTo.cpp
 *
 *  Created on: Jan 9, 2012
 *      Author: mriedel
 */

#include "DynamicFlyTo.hpp"

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::DynamicFlyTo, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

DynamicFlyTo::DynamicFlyTo()
	: Behavior("tk_be_common/DynamicFlyTo", BehaviorType::Air)
{

}

void DynamicFlyTo::initialize()
{
	tFlyToDestination = addOption<Position3D>("tFlyToDestination",
			"Specifies the Destination of the DynamicFlyTo Behavior",
			Position3D(0.0,0.0,-1.0), false, false);
	tFlyToVelocity = addOption<double>("tFlyToVelocity",
			"Defines the Velocity of the DynamicFlyTo Behavior",
			5.0, false, false);
	tFlyToInnerDestinationRadius = addOption<double>("tFlyToInnerDestinationRadius",
			"Defines the Radius from the Destination, at which the DynamicFlyTo Behavior turns invalid",
			0.05, false, false);
	tFlyToOuterDestinationRadius = addOption<double>("tFlyToOuterDestinationRadius",
			"Defines the Radius at which the QC linearly decreases speed.",
			0.5, false, false);

	//parameterInitialized = true; // not parameter Initialized by default.
}

void DynamicFlyTo::destroy()
{

}

bool DynamicFlyTo::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	yawAngle = currentState.getEulerRPY()(2);

	outerRadiusAcceleration = (tFlyToVelocity->getValue() * tFlyToVelocity->getValue()) /
			( 2 * tFlyToOuterDestinationRadius->getValue() );
	//ROS_INFO("DynamicFlyTo: Storing yawAngle: %f ", yawAngle);
	return true;
}

void DynamicFlyTo::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void DynamicFlyTo::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void DynamicFlyTo::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void DynamicFlyTo::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	Vector3D direction = tFlyToDestination->getValue() - currentState.position;
	generatedTrajInput.setVelocity( direction.normalized() * tFlyToVelocity->getValue() );
	generatedTrajInput.setYawAngle( yawAngle );
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void DynamicFlyTo::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	Vector3D direction = tFlyToDestination->getValue() - currentState.position;
	// Linearly slow down from tFlyToOuterDestinationRadius.
	double velocityFactor = 1.0;
	Acceleration3D acceleration = Acceleration3D::Zero();
	if (direction.norm() < tFlyToOuterDestinationRadius->getValue()) {
		// Slow down
		//ROS_INFO("Breaking!");
		velocityFactor = direction.norm() / tFlyToOuterDestinationRadius->getValue(); // between 0.0 - 1.0
		acceleration = direction.normalized() * outerRadiusAcceleration;
	}

	generatedTrajInput.setVelocity( direction.normalized() * tFlyToVelocity->getValue() * velocityFactor,
			acceleration);
	generatedTrajInput.setYawAngle( yawAngle );
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void DynamicFlyTo::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawAngle( yawAngle );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool DynamicFlyTo::isValid(const TKState& currentState) const
{
	Vector3D direction = tFlyToDestination->getValue() - currentState.position;
	return direction.norm() > tFlyToInnerDestinationRadius->getValue();
}


} /* namespace telekyb_behavior */

