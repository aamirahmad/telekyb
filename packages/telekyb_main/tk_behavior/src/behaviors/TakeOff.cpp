/*
 * TakeOff.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include <behaviors/TakeOff.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS(telekyb_behavior::TakeOff, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

TakeOff::TakeOff()
	: Behavior("tk_behavior/TakeOff", BehaviorType::TakeOff)
{

}

void TakeOff::initialize()
{
	tTakeOffDestination = addOption<Position3D>("tTakeOffDestination",
			"Specifies the Destination of the TakeOff Behavior",
			Position3D(0.0,0.0,-1.0), false, false);
	tTakeOffVelocity = addOption<double>("tTakeOffVelocity",
			"Defines the Velocity of the TakeOff Behavior",
			0.3, false, false);
	tTakeOffDestinationRadius = addOption<double>("tTakeOffDestinationRadius",
			"Defines the Radius from the Destination, at which the TakeOff Behavior turns invalid",
			0.1, false, false);
	tTakeOffVertically = addOption<bool>("tTakeOffVertically",
			"Take off vertically to the z-Component of the Destination",
			false, false, false);
	tTakeOffWithYawRate = addOption<bool>("tTakeOffWithYawRate",
			"Take off controlling the yaw rate instead of the yaw angle",
			false, false, false);
	tTakeOffInPosition = addOption<bool>("tTakeOffInPosition",
			"Take off giving only the final position to the controller",
			false, false, false);
	//parameterInitialized = true; // not parameter Initialized by default.
}

void TakeOff::destroy()
{

}

bool TakeOff::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// TODO: Out of correctness we should check if currentState is too close to destination.
	// -> on the other hand it would return false right after in the trajStep.
	if (tTakeOffVertically->getValue() ) {
		if (tTakeOffWithYawRate->getValue()) {
				// We don't care about the x and y coordinates since we need only the height
				actualTakeOffDestination(0) = 0.0;
				actualTakeOffDestination(1) = 0.0;
				actualTakeOffDestination(2) = tTakeOffDestination->getValue()(2);
		} else {
				actualTakeOffDestination = currentState.position;
				actualTakeOffDestination(2) = tTakeOffDestination->getValue()(2);
		}
	} else {
		actualTakeOffDestination = tTakeOffDestination->getValue();
	}

	yawAngle = currentState.getEulerRPY()(2);
	return true;
}

void TakeOff::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void TakeOff::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void TakeOff::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void TakeOff::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void TakeOff::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
    if (tTakeOffWithYawRate->getValue()) {
			Vector3D velocity = Velocity3D::Zero();
			velocity(2) = -tTakeOffVelocity->getValue();
			generatedTrajInput.setVelocity(velocity);
			generatedTrajInput.setYawRate( 0 );
	} else {
		Velocity3D direction = actualTakeOffDestination - currentState.position;	
		generatedTrajInput.setVelocity( direction.normalized() * tTakeOffVelocity->getValue() );
		generatedTrajInput.setYawAngle( yawAngle );
	}
	if (tTakeOffInPosition->getValue()){
		generatedTrajInput.setPosition(actualTakeOffDestination);
	}

}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void TakeOff::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool TakeOff::isValid(const TKState& currentState) const
{
	Vector3D direction;
	if (tTakeOffWithYawRate->getValue()) {
			direction = Vector3D::Zero();
			direction(2) = actualTakeOffDestination(2) - currentState.position(2);
	} else {
			direction = actualTakeOffDestination - currentState.position;
	}
	return direction.norm() > tTakeOffDestinationRadius->getValue();
}

}
