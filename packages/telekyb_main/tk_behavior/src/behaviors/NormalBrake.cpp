/*
 * NormalBrake.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: mriedel
 */

#include <behaviors/NormalBrake.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::NormalBrake, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

NormalBrake::NormalBrake()
	: Behavior("tk_behavior/NormalBrake", BehaviorType::Air)
{

}

void NormalBrake::initialize()
{
	// NormalBrake MUST operate on default Parameters
	tBrakeMinVelocity = addOption<double>("tBrakeMinVelocity",
			"Minimum Velocity for NormalBrake Behavior.",0.1, false, false);
	tBrakeWhitYawRate = addOption<bool>("tBrakeWhitYawRate",
			"Brake controlling the yaw rate instead of the yaw angle",
			false, false, false);
	tBrakeInPosition = addOption<bool>("tBrakeInPosition",
			"Brake giving only the final position to the controller",
			false, false, false);

	// This MUST be true!
	parameterInitialized = true;
}

void NormalBrake::destroy()
{

}

/** NORMALBRAKE SHOULD NEVER RETURN FALSE! **/
bool NormalBrake::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	yawAngle = currentState.getEulerRPY()(2);
	desiredPosition = currentState.position;

	return true;
}

void NormalBrake::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void NormalBrake::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void NormalBrake::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void NormalBrake::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void NormalBrake::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	if (tBrakeWhitYawRate->getValue()) {
			generatedTrajInput.setYawRate( 0 );
	} else {
			generatedTrajInput.setYawAngle( yawAngle );
	}
	if (tBrakeInPosition->getValue()){
		generatedTrajInput.setPosition(desiredPosition);
	}
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void NormalBrake::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool NormalBrake::isValid(const TKState& currentState) const
{
	return currentState.linVelocity.norm() > tBrakeMinVelocity->getValue();
}



}
