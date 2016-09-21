/*
 * EmergencyLand.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <behaviors/EmergencyLand.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::EmergencyLand, TELEKYB_NAMESPACE::Behavior);

#define MAX_HEIGHT -0.3 // Z DOWN!
#define LANDING_VELOCITY 0.7
#define LANDING_POS_Z 0.0

namespace telekyb_behavior {

EmergencyLand::EmergencyLand()
	: Behavior("tk_behavior/EmergencyLand", BehaviorType::Land)
{

}

void EmergencyLand::initialize()
{
	parameterInitialized = true;
}

void EmergencyLand::destroy()
{

}

bool EmergencyLand::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	landingPosition = currentState.position;
	landingPosition(2) = LANDING_POS_Z;
	return true;
}

void EmergencyLand::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void EmergencyLand::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void EmergencyLand::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void EmergencyLand::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void EmergencyLand::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{

	Vector3D direction = landingPosition - currentState.position;
	generatedTrajInput.setVelocity( direction.normalized() * LANDING_VELOCITY );
	generatedTrajInput.setYawRate(0.0);
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void EmergencyLand::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Velocity 0
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate(0.0);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool EmergencyLand::isValid(const TKState& currentState) const
{
	// Break condition
	return currentState.position(2) < MAX_HEIGHT;
}

}
