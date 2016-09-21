/*
 * Land.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: mriedel
 */

#include <behaviors/Land.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS(telekyb_behavior::Land, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

Land::Land()
	: Behavior("tk_behavior/Land", BehaviorType::Land)
{

}

void Land::initialize()
{
	tLandDestination = addOption<Position3D>("tLandDestination",
			"Specifies the Destination of the Land Behavior",
			Position3D(0.0,0.0,0.0), false, false);
	tLandVelocity = addOption<double>("tLandVelocity",
			"Defines the Velocity of the Land Behavior",
			0.5, false, false);
	tLandDestinationHeight = addOption<double>("tLandDestinationHeight",
			"Defines the Height from the Ground, at which the Land Behavior turns invalid. (Beware Z is down!)",
			-0.25, false, false);
	tLandVertically = addOption<bool>("tLandVertically",
			"Land vertically to the z-Component of the Destination",
			true, false, false);
	tLandWithYawRate = addOption<bool>("tLandWithYawRate",
			"Land controlling the yaw rate instead of the yaw angle",
			false, false, false);
	tLandInPosition = addOption<bool>("tLandInPosition",
			"Land giving only the final position to the controller",
			false, false, false);

	// Not parameter Initalized by default
	// parameterInitialized = true;
}

void Land::destroy()
{

}

bool Land::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	if (tLandVertically->getValue() ) {
		actualLandDestination = currentState.position;
		actualLandDestination(2) = tLandDestination->getValue()(2);
	} else {
		actualLandDestination = tLandDestination->getValue();
	}

	yawAngle = currentState.getEulerRPY()(2);
	return true;
}

void Land::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void Land::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void Land::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void Land::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void Land::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	Vector3D direction = actualLandDestination - currentState.position;
	generatedTrajInput.setVelocity( direction.normalized() * tLandVelocity->getValue() );
	if (tLandWithYawRate->getValue()) {
			generatedTrajInput.setYawRate( 0 );
	} else {
			generatedTrajInput.setYawAngle( yawAngle );
	}
	if (tLandInPosition->getValue()){
		generatedTrajInput.setPosition(actualLandDestination);
	}
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void Land::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool Land::isValid(const TKState& currentState) const
{
	return currentState.position(2) < tLandDestination->getValue()(2) + tLandDestinationHeight->getValue();
}


}
